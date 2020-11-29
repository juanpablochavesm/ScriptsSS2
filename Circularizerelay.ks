lock throttle to t.
wait until eta:apoapsis < 300.
set warp to 3.
wait until eta:apoapsis < 50.
set warp to 0.
wait 30.
sas on.
wait 2.
set sasmode to "prograde".
wait until eta:apoapsis < 10.
until ship:periapsis > 200000 {
		set t to 1.
}
set t to 0.
set warp to 3.
wait until eta:periapsis < 300.
set warp to 3.
wait until eta:periapsis < 50.
set warp to 0.
wait 30.
sas on.
wait 2.
set sasmode to "retrograde".
wait until eta:periapsis < 10.
until ship:apoapsis < 202000 {
		set t to 1.
}
set t to 0.
unlock steering. sas off. unlock throttle.
SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.
