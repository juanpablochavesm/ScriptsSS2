declare parameter orbith.
clearscreen.
print "Target orbit: " + orbith + " km".
set finalspeed to (body:mu*((0.002/(orbith+body:radius))-(0.001/(orbith+body:radius))))^0.5. 	//finalspeed es velocidad orbital final
Print "Final Orbital Speed: " + round(finalspeed,1) + "m/s".
lock acc to ship:maxthrust/ship:mass.
set throt to 0.
lock throttle to throt.
if ship:apoapsis/1000 < orbith  {
	print "Proceeding to raise orbit.".
	set SMaj to (ship:periapsis + orbith*1000 + 2*body:radius)/2.		//calcula semi major de transferencia
	set transferspeed to (body:mu*(2/(ship:periapsis+body:radius)-1/SMaj))^0.5. 	//Velocidad de transferencia
	print "Transfer speed: " + round(transferspeed,1) + " m/s".
	set perispeed to (body:mu*(2/(ship:periapsis+body:radius)-2/(ship:periapsis + ship:apoapsis+2*body:radius)))^0.5.
	print "Periapsis speed: " + round(perispeed,1) + " m/s".
	set burntime to (transferspeed-perispeed)/acc.
	set q to 1.
	if burntime < 10  {
		set q to 10/burntime.
		set burntime to burntime*q.
	}
	lock diferencia to eta:periapsis - burntime*0.5.
	print round(diferencia) + " seconds to burn start.".
	wait until diferencia < 50.
	Set warp to 0.
	wait 30.
	sas on.
	wait 1.
	set sasmode to "prograde".
	wait until diferencia < 10.
	print "10 seconds to burn start".
	sas on.
	wait 1.
	set sasmode to "prograde".
	wait until diferencia < 2.
	set sasmode to "prograde".
	wait until diferencia < 0.
	print "Burn start.".
	until ship:apoapsis > (orbith*1000)   {
		set throt to min(1,20*(1-ship:apoapsis/(orbith*1001)))/q.
	}
	set throt to 0.
	print "Periapsis burn complete. Proceeding to circularize.".
	wait 1.
	sas off.
	set apospeed to (body:mu*(2/(ship:apoapsis+body:radius)-1/SMaj))^0.5.
	set burntime to (finalspeed-apospeed)/acc.
	if burntime < 10  {
		set q to 10/burntime.
		set burntime to burntime*q.
	}
	lock diferencia to eta:apoapsis - burntime*0.5.
	print round(diferencia) + " seconds to burn start.".
	wait until diferencia < 50.
	Set warp to 0.
	wait 30.
	sas on.
	wait 1.
	set sasmode to "prograde".
	wait until diferencia < 10.
	print "10 seconds to burn start".
	sas on.
	wait 1.
	set sasmode to "prograde".
	wait until diferencia < 2.
	set sasmode to "prograde".
	wait until diferencia < 0.
	print "Burn start".
	until ship:periapsis > (orbith*1000)   {
		if ship:apoapsis-ship:periapsis  < 50  {
			break.
		}
		if ship:apoapsis-orbith*1000  > 370  {
			break.
		}
		set throt to min(1,20*(1-ship:periapsis/(orbith*1001)))/q.
	}
	set throt to 0.
	print "Circularization complete.".
	wait 1.
	sas off.
	SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.

}
else if ship:periapsis/1000 > orbith  {
	print "Proceeding to lower orbit.".
	set SMaj to (ship:apoapsis + orbith*1000 + 1200000)/2.		//calcula semi major de transferencia
	set transferspeed to (body:mu*(2/(ship:apoapsis+600000)-1/SMaj))^0.5. 	//Velocidad de transferencia
	print "Transfer speed: " + round(transferspeed,1) + " m/s".
	set apospeed to (body:mu*(2/(ship:apoapsis+600000)-2/(ship:periapsis + ship:apoapsis+1200000)))^0.5.
	print "Apoapsis speed: " + round(perispeed,1) + " m/s".
	set burntime to (apospeed-transferspeed)/acc.
	set q to 1.
	if burntime < 10  {
		set q to 10/burntime.
		set burntime to burntime*q.
	}
	lock diferencia to eta:apoapsis - burntime*0.5.
	print round(diferencia) + " seconds to burn start.".
	wait until diferencia < 60.
	Set warp to 0.
	wait 30.
	sas on.
	wait 1.
	set sasmode to "retrograde".
	wait until diferencia < 10.
	print "10 seconds to burn start".
	sas on.
	wait 1.
	set sasmode to "retrograde".
	wait until diferencia < 2.
	set sasmode to "retrograde".
	wait until diferencia < 0.
	print "Burn start.".
	until ship:velocity:orbit:mag-transferspeed < acc*0.1/q   {
		set throt to 1/q.
	}
	until ship:periapsis  < (orbith*1000)+50   {
		set throt to 0.5/acc.
	}
	set throt to 0.
	print "Apoapsis burn complete. Proceeding to circularize.".
	wait 1.
	sas off.
	set perispeed to (body:mu*(2/(ship:periapsis+600000)-1/SMaj))^0.5.
	set burntime to (perispeed-finalspeed)/acc.
	if burntime < 10  {
		set q to 10/burntime.
		set burntime to burntime*q.
	}
	lock diferencia to eta:periapsis - burntime*0.5.
	print round(diferencia) + " seconds to burn start.".
	wait until diferencia < 60.
	Set warp to 0.
	wait 30.
	sas on.
	wait 1.
	set sasmode to "retrograde".
	wait until diferencia < 10.
	print "10 seconds to burn start".
	sas on.
	wait 1.
	set sasmode to "retrograde".
	wait until diferencia < 2.
	set sasmode to "retrograde".
	wait until diferencia < 0.
	print "Burn start".
	until ship:velocity:orbit:mag-finalspeed < acc*0.1/q   {
		set throt to 1/q.
	}
	until ship:apoapsis  < (orbith*1000)+50   {
		set throt to 0.5/acc.
	}
	set throt to 0.
	print "Circularization complete.".
	wait 1.
	sas off.
	SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.
}
else if ship:periapsis/1000 < orbith  {

}
