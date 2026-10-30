package com.synthwomb.whip

import org.json.JSONObject
import java.util.Locale

// What WHIP takes: the same calls as RITA and his PC controller.
class Robot(private val http: Http) : RobotApi {
    private fun n(v: Float) = String.format(Locale.ROOT, "%.2f", v)

    override fun connect(pin: String) {
        http.get("/unlock?pw=$pin")
        http.get("/mode?m=web")
    }
    override fun drive(forward: Float, side: Float, turn: Float) {
        http.get("/web?cmd=drive&f=${n(forward)}&l=${n(side)}&t=${n(turn)}")
    }
    override fun stop() { http.get("/web?cmd=stop") }
    override fun tiles(s: JSONObject) = listOf(
        "Wall" to Tiles.cm(s.opt("distance")),
        "Pitch" to "${s.optDouble("pitch").toInt()}\u00b0",
        "Roll" to "${s.optDouble("roll").toInt()}\u00b0",
        "Gait" to s.optString("gait", "--"),
        "Height" to s.optString("height", "--"),
    )

    override val buttons = listOf(
        "OBSTACLE" to { http.get("/mode?m=obstacle"); Unit },
        "WEB" to { http.get("/mode?m=web"); Unit },
        "HELLO" to { http.get("/act?do=hello"); Unit },
        "DANCE" to { http.get("/act?do=dance"); Unit },
        "REST" to { http.get("/act?do=rest"); Unit },
    )
}
