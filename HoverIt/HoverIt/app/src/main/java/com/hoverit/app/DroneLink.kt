package com.hoverit.app

import android.content.Context
import android.net.ConnectivityManager
import android.net.wifi.WifiManager
import android.os.SystemClock
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress
import java.net.InetSocketAddress
import java.net.SocketTimeoutException

/**
 * The drone's flight modes (FlightModes.h), in the drone's order: index = the
 * 2-bit mode number in the flags byte.
 */
enum class FlightMode(val title: String, val blurb: String?) {
    STABILIZE("Stabilize", "manual throttle"),
    LOITER("Loiter", "50% stick holds height"),
    LAND("Land", "descend + disarm"),
    AUTO("Autonomous", null);

    /** Throttle stick does nothing on the drone in this mode. */
    val throttleLocked get() = this == LAND || this == AUTO
    /** No stick does anything on the drone in this mode. */
    val sticksLocked get() = this == AUTO
}

/** What the drone says it is doing (FlightStatus in FlightModeCommon.h, same order). */
val FLIGHT_STATUS = listOf(
    "on the ground", "flying", "holding height", "climbing", "descending", "landing",
    "landed", "spooling up", "climbing to 50 cm", "square 1/4", "square 2/4", "square 3/4",
    "square 4/4", "baro training",
)
const val STATUS_IDLE = 0
const val STATUS_LANDED = 6

/**
 * Everything the phone sends to the drone, one frame.
 * Sticks, sliders and battery are 8 bit (0-255), arm / training are 1 bit.
 */
data class ControlFrame(
    val throttle: Int = 0,      // left stick Y   (0 = bottom)
    val yaw: Int = 128,         // left stick X   (128 = centre)
    val pitch: Int = 128,       // right stick Y  (128 = centre, 255 = forward)
    val roll: Int = 128,        // right stick X  (128 = centre, 255 = right)
    val kp: Int = 0,
    val ki: Int = 0,
    val kd: Int = 0,
    val ff: Int = 0,
    val loop: Int = 0,          // PID loop the sliders tune (see PID_LOOPS)
    val armed: Boolean = false,
    val training: Boolean = false,  // Options > Enable Baro (bit1)
    val tuning: Boolean = false,    // Tuning sliders enabled
    val mode: FlightMode = FlightMode.STABILIZE,
)

/** The drone's view of the flight: the mode it flies, what it does, its height. */
data class FlightState(
    val mode: FlightMode = FlightMode.STABILIZE,
    val status: Int = STATUS_IDLE,
    val heightCm: Int = 0,
    val training: Boolean = false,
) {
    val statusText get() = FLIGHT_STATUS.getOrElse(status) { "?" }
    /** Motors running. */
    val airborne get() = status != STATUS_IDLE && status != STATUS_LANDED
}

/**
 * The PID loops the drone knows, in the drone's order (index = loop byte):
 * 0 RP_RATE (roll + pitch rate, one set of gains), 1 YAW_RATE, 2 VERT_VEL
 * (vertical velocity). The angle and height loops are fixed on the drone.
 */
val PID_LOOPS = listOf("ROLL+PITCH", "YAW RATE", "VERT VEL")

/** One loop's gains as the drone reports them, 0-255 each. */
data class DroneGains(val loop: Int, val kp: Int, val ki: Int, val kd: Int, val ff: Int)

/**
 * UDP link to the ESP32 (HoverItRx.h on the drone).
 *
 * Phone -> drone, port 4210, 50 Hz, 14 bytes:
 *   [0] 'H'  [1] seq  [2] throttle [3] yaw [4] pitch [5] roll
 *   [6] kp   [7] ki   [8] kd       [9] ff
 *   [10] loop: which PID loop kp..ff are for (0-2, PID_LOOPS)
 *   [11] flags: bit0 armed, bit1 baro height on, bit2 tuning sliders enabled,
 *        bits4-5 flight mode (FlightMode ordinal)
 *   [12] protocol version (3)
 *   [13] XOR of bytes 0..12
 *
 * Drone -> phone, port 4211, 10 Hz, 12 bytes:
 *   [0] 'D'  [1] seq  [2] battery 0-255
 *   [3] flags: bit0 armed, bit1 barometer in use for height, bits4-5 mode it flies
 *   [4] loop [5] kp [6] ki [7] kd [8] ff   the drone's gains for that loop
 *   [9] height in cm / 2   [10] status (FLIGHT_STATUS index)
 *   [11] XOR of 0..10
 *
 * Discovery: until the drone hears the phone it broadcasts its telemetry, so the
 * phone learns the drone's IP from the first packet. Until then the phone
 * broadcasts its frames (always disarmed) so either side can find the other.
 * The drone counts as disconnected when no telemetry arrives for 1 s.
 */
class DroneLink(context: Context, private val scope: CoroutineScope) {

    companion object {
        const val DRONE_PORT = 4210
        const val PHONE_PORT = 4211
        const val SEND_PERIOD_MS = 20L
        const val TIMEOUT_MS = 1000L
        const val PROTOCOL_VERSION = 3
        const val TELEMETRY_BYTES = 12
    }

    private val wifi = context.applicationContext.getSystemService(Context.WIFI_SERVICE) as WifiManager
    private val connectivity = context.applicationContext.getSystemService(Context.CONNECTIVITY_SERVICE) as ConnectivityManager

    /**
     * Send through the network the drone is actually on. On the drone's own
     * network (HOVERIT_SOFTAP, no internet) Android would otherwise send over
     * mobile data, and with the phone hotspot on while the phone is also on
     * home Wi-Fi, a blind "use Wi-Fi" would pick the wrong one. So: once the
     * drone's address is known, pin the socket to the network whose subnet
     * contains it. No match (the phone's own hotspot) = leave it to normal
     * routing, which already reaches hotspot clients.
     */
    @Suppress("DEPRECATION")
    private fun bindToNetworkOf(addr: InetAddress, s: DatagramSocket) {
        try {
            for (n in connectivity.allNetworks) {
                val lp = connectivity.getLinkProperties(n) ?: continue
                if (lp.linkAddresses.any { sameSubnet(it.address, it.prefixLength, addr) }) {
                    n.bindSocket(s)
                    return
                }
            }
        } catch (_: Exception) {
            // not bound: normal routing
        }
    }

    private fun sameSubnet(a: InetAddress, prefix: Int, b: InetAddress): Boolean {
        val x = a.address
        val y = b.address
        if (x.size != y.size) return false
        var bits = prefix
        for (i in x.indices) {
            if (bits <= 0) return true
            val mask = if (bits >= 8) 0xFF else (0xFF shl (8 - bits)) and 0xFF
            if ((x[i].toInt() and mask) != (y[i].toInt() and mask)) return false
            bits -= 8
        }
        return true
    }

    /** Written by the UI thread, read by the sender. */
    @Volatile var frame = ControlFrame()

    private val _enabled = MutableStateFlow(false)
    val enabled: StateFlow<Boolean> = _enabled.asStateFlow()

    private val _connected = MutableStateFlow(false)
    val connected: StateFlow<Boolean> = _connected.asStateFlow()

    private val _battery = MutableStateFlow(0)
    val battery: StateFlow<Int> = _battery.asStateFlow()

    private val _droneArmed = MutableStateFlow(false)
    val droneArmed: StateFlow<Boolean> = _droneArmed.asStateFlow()

    /** The drone's gains for the loop it reports (null until it does). */
    private val _gains = MutableStateFlow<DroneGains?>(null)
    val gains: StateFlow<DroneGains?> = _gains.asStateFlow()

    private val _flight = MutableStateFlow(FlightState())
    val flight: StateFlow<FlightState> = _flight.asStateFlow()

    private val _droneAddress = MutableStateFlow<String?>(null)
    val droneAddress: StateFlow<String?> = _droneAddress.asStateFlow()

    @Volatile private var drone: InetAddress? = null
    @Volatile private var lastRxMs = 0L
    private var socket: DatagramSocket? = null
    private var multicastLock: WifiManager.MulticastLock? = null
    private val jobs = mutableListOf<Job>()

    fun start() {
        if (_enabled.value) return
        _enabled.value = true
        multicastLock = wifi.createMulticastLock("hoverit").apply {
            setReferenceCounted(false)
            acquire()
        }
        jobs += scope.launch(Dispatchers.IO) {
            val s = try {
                DatagramSocket(null).apply {
                    reuseAddress = true
                    broadcast = true
                    soTimeout = 200
                    bind(InetSocketAddress(PHONE_PORT))
                }
            } catch (e: Exception) {
                launch(Dispatchers.Main) { stop() }
                return@launch
            }
            socket = s
            launch { receiveLoop(s) }
            launch { sendLoop(s) }
        }
        jobs += scope.launch { watchdogLoop() }
    }

    fun stop() {
        if (!_enabled.value) return
        _enabled.value = false
        jobs.forEach { it.cancel() }
        jobs.clear()
        socket?.close()
        socket = null
        multicastLock?.let { if (it.isHeld) it.release() }
        multicastLock = null
        drone = null
        _droneAddress.value = null
        _connected.value = false
        _droneArmed.value = false
        _gains.value = null
        _flight.value = FlightState()
    }

    private suspend fun receiveLoop(s: DatagramSocket) {
        val buf = ByteArray(64)
        val pkt = DatagramPacket(buf, buf.size)
        while (scope.isActive && _enabled.value && !s.isClosed) {
            try {
                pkt.length = buf.size
                s.receive(pkt)
            } catch (e: SocketTimeoutException) {
                continue
            } catch (e: Exception) {
                if (s.isClosed) break else continue
            }
            val n = TELEMETRY_BYTES
            if (pkt.length != n || buf[0] != 'D'.code.toByte()) continue
            var x = 0
            for (i in 0 until n - 1) x = x xor (buf[i].toInt() and 0xFF)
            if (x != (buf[n - 1].toInt() and 0xFF)) continue

            if (drone != pkt.address) {
                drone = pkt.address
                bindToNetworkOf(pkt.address, s)
                _droneAddress.value = pkt.address.hostAddress
            }
            lastRxMs = SystemClock.elapsedRealtime()
            _battery.value = buf[2].toInt() and 0xFF
            val flags = buf[3].toInt() and 0xFF
            _droneArmed.value = (flags and 0x01) != 0
            _flight.value = FlightState(
                mode = FlightMode.entries[(flags shr 4) and 0x03],
                status = buf[10].toInt() and 0xFF,
                heightCm = (buf[9].toInt() and 0xFF) * 2,
                training = (flags and 0x02) != 0,
            )
            _gains.value = DroneGains(
                loop = buf[4].toInt() and 0xFF,
                kp = buf[5].toInt() and 0xFF,
                ki = buf[6].toInt() and 0xFF,
                kd = buf[7].toInt() and 0xFF,
                ff = buf[8].toInt() and 0xFF,
            )
            _connected.value = true
        }
    }

    private suspend fun sendLoop(s: DatagramSocket) {
        val out = ByteArray(14)
        val broadcast = InetAddress.getByName("255.255.255.255")
        var seq = 0
        while (scope.isActive && _enabled.value && !s.isClosed) {
            val f = frame
            val target = drone
            // never send "armed" until we know exactly which drone we talk to
            val armed = f.armed && target != null && _connected.value
            out[0] = 'H'.code.toByte()
            out[1] = seq.toByte()
            out[2] = f.throttle.toByte()
            out[3] = f.yaw.toByte()
            out[4] = f.pitch.toByte()
            out[5] = f.roll.toByte()
            out[6] = f.kp.toByte()
            out[7] = f.ki.toByte()
            out[8] = f.kd.toByte()
            out[9] = f.ff.toByte()
            out[10] = f.loop.toByte()
            out[11] = ((if (armed) 0x01 else 0) or
                    (if (f.training) 0x02 else 0) or
                    (if (f.tuning) 0x04 else 0) or
                    ((f.mode.ordinal and 0x03) shl 4)).toByte()
            out[12] = PROTOCOL_VERSION.toByte()
            var x = 0
            for (i in 0 until 13) x = x xor (out[i].toInt() and 0xFF)
            out[13] = x.toByte()
            try {
                s.send(DatagramPacket(out, out.size, target ?: broadcast, DRONE_PORT))
            } catch (_: Exception) {
                // no network yet (hotspot off, Wi-Fi switching): keep trying
            }
            seq = (seq + 1) and 0xFF
            delay(SEND_PERIOD_MS)
        }
    }

    private suspend fun watchdogLoop() {
        while (scope.isActive && _enabled.value) {
            if (_connected.value && SystemClock.elapsedRealtime() - lastRxMs > TIMEOUT_MS) {
                _connected.value = false
                _droneArmed.value = false
            }
            delay(100)
        }
    }
}
