declare parameter inclination.
FUNCTION fburntime {
  PARAMETER dV.

  LIST ENGINES IN en.

  LOCAL f IS en[0]:MAXTHRUST * 1000.  // Engine Thrust (kg * m/s²)
  LOCAL m IS SHIP:MASS * 1000.        // Starting mass (kg)
  LOCAL e IS CONSTANT():E.            // Base of natural log
  LOCAL p IS en[0]:ISP.               // Engine ISP (s)
  LOCAL g IS 9.80665.                 // Gravitational acceleration constant (m/s²)

  RETURN g * m * p * (1 - e^(-dV/(g*p))) / f.
}
clearscreen.
set inclination to mod(inclination,360).
set orbinclination to inclination.
print "Target inclination at " + inclination + " degrees heading.".
if inclination <= 90 {
	set x to arctan(sin(90-inclination)*175/(900-cos(90-inclination)*175)).
	if inclination-x < 0 {
		set inclination to inclination + 360.
	}
	set inclination to inclination-x.
	print "Pitchover manouver to " + round(inclination)+ " degrees inclination.".
}
else if inclination >= 270  {
	set x to arctan(sin(inclination-270)*175/(900-cos(inclination-270)*175)).
	set inclination to inclination-x.
	print "Pitchover manouver to " + round(inclination)+ " degrees inclination.".
}
else if inclination > 180 {
	set x to arctan(sin(inclination-90)*175/(900-cos(inclination-90)*175)).
	set inclination to inclination+x.
	print "Pitchover manouver to " + round(inclination)+ " degrees inclination.".
}
else {
	set x to arctan(sin(270-inclination)*175/(900-cos(270-inclination)*175)).
	set inclination to inclination+x.
	print "Pitchover manouver to " + round(inclination) + " degrees heading.".
}
lock throttle to 1.
lock x to ship:altitude/1000.
lock b to -0.0006*x^3+0.06667*x^2-3.2931*x+90.
set a to heading(inclination,b).
lock steering to a.
stage.
until ship:altitude>10000 {
	set a to heading(inclination,b-1).
	wait 1.
}
lock steering to srfprograde.
wait until ship:apoapsis>70500.
lock throttle to 0.
wait 1.
stage.
set acc to ship:maxthrust/ship:mass.
lock v to ship:velocity:orbit:mag.
lock difv to 2294-v.
lock circburntime to difv/acc.
wait until eta:apoapsis<circburntime*1.
set api to ship:apoapsis.
lock steering to heading(orbinclination,0).
wait until eta:apoapsis<fburntime(difv)*0.65.
set throt to 1.
lock throttle to throt.
set accel to acc.
until ship:periapsis>70025  {
	set corr to eta:apoapsis-fburntime(difv)*0.65.
	if corr > 10 {
	break.
	}
	set throt to min(1,max(0,throt-corr*0.1)).
	wait 0.1.
}
until ship:periapsis>70025  {
	set throt to 10/accel.
}
panels on.
sas off.
rcs off.
print "Orbit achieved".
SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.	


