//FUNCIONES

FUNCTION fburntime {
  PARAMETER dV.

  LOCAL e IS CONSTANT():E.            // Base of natural log

  RETURN 9.80665 * SHIP:MASS * 305 * (1 - e^(-dV/(9.80665*305))) / ship:maxthrust.
}
FUNCTION perispeed {
  set perispeedx to (body:mu*(2/(ship:periapsis+body:radius)-2/(ship:periapsis + ship:apoapsis+body:radius*2)))^0.5.
  return perispeedx.
}
FUNCTION apospeed {
  set apospeedx to (body:mu*(2/(ship:apoapsis+body:radius)-2/(ship:periapsis + ship:apoapsis+body:radius*2)))^0.5.
  return apospeedx.
}

// Variables ajustables de prueba:

set DivisorYawEnRunway to 20.
set PitchAtClimb to 32.
set MachForPitchDown to 0.75.
set PitchAtMach1 to 11.
set MachForPitchUp to 1.5.
set PitchTransonic to 22.
set AltitudeForSpeedrun to 12000.
set PitchForSpeedrun to 18.
set MachForZoom to 4.
set PitchForZoom to 24.
set PitchForMECO to 16.
set TargetApoapsis to 70500.
set CirculariBurnFactor to 0.55.     // Factor que multiplica al ETA para empezar Second Burn

// Configuracion inicial:

SAS OFF.
RCS off.
Lights off.
set throt to 0.
lock throttle to throt.
panels off.
set KSCwest to latlng( -0.048597000539, -74.72335052490).
set KSCeast to latlng( -0.05028, -74.48821).
lock currentpos to SHIP:GEOPOSITION.
lock rightrotation to ship:facing*r(0,90,0).


// VECTORES Y ANGULOS

lock right to rightrotation:vector. //right and left are directly along wings
lock left to (-1)*right.
lock ups to ship:up:vector. //up and down are skyward and groundward
lock down to (-1)*up.
lock fore to ship:facing:vector. //fore and aft point to the nose and tail
lock aft to (-1)*fore.
lock righthor to vcrs(up:vector,fore). //right and left horizons
lock lefthor to (-1)*righthor.
lock forehor to vcrs(righthor,up:vector). //forward and backward horizons
lock afthor to (-1)*forehor.
lock top to vcrs(fore,right). //above the cockpit, through the floor
lock bottom to (-1)*top.
lock absaoa to vang(fore,srfprograde:vector). //absolute angle of attack
lock aoa to vang(top,srfprograde:vector)-90. //pitch component of angle of attack
lock sideslip to vang(right,srfprograde:vector)-90. //yaw component of aoa
lock rollangle to vang(right,righthor)*((90-vang(top,righthor))/abs(90-vang(top,righthor))). //roll angle, 0 at level flight
lock pitchangle to vang(fore,forehor)*((90-vang(fore,up:vector))/abs(90-vang(fore,up:vector))). //pitch angle, 0 at level flight
lock east to vcrs(up:vector, north:vector).

clearscreen.

// DESPEGUE

sas on.
print "Mode: Takeoff" at (0,1).
set hea to heading(90,0).
lock steering to hea.
set throt to 1.
stage.
until ship:velocity:surface:mag > 70 {
  if abs(vang(righthor,fore)) > 0.5 {
    set ship:control:yaw to (vang(righthor,fore)-90)/DivisorYawEnRunway.
  }
  else {
    set ship:control:yaw to 0.
  }
  wait 0.05.
}


// PITCH UP

set gearflag to 0.
print "Mode: Pitch up manouver" at (0,1).
set targetp to 0.
until targetp > PitchAtClimb-(ship:mass-90)/3 {
  set targetp to targetp + 1.
  set hea to lookdirup(heading(90,targetp):vector,up:vector).
  wait 0.5.
  if targetp > 10 {
    if gearflag = 0 {
      gear off.
      set gearflag to 1.
    }
  }
}

// MACH 1 PITCHDOWN

lock mach to airspeed/323.
wait until mach > MachForPitchDown.
lock hea to lookdirup(heading(90,targetp):vector,up:vector).
clearscreen.
print "Mode: Mach 1 Aerodynamics" at (0,1).
until targetp < PitchAtMach1 {
  set targetp to targetp - 1.
  wait 0.7.
}

// TRANSONIC PITCHUP

wait until mach > MachForPitchUp.
clearscreen.
print "Mode: Transonic Climb" at (0,1).
until targetp > PitchTransonic {
  wait 0.2.
  set targetp to targetp + 0.2.
}

// SPEEDRUN

wait until ship:altitude > AltitudeForSpeedrun.
clearscreen.
print "Mode: Speedrun" at (0,1).
until targetp < PitchForSpeedrun {
  wait 0.4.
  set targetp to targetp - 0.2.
}

// ZOOMCLIMB

lock TWR to ship:maxthrust/(ship:MASS*9.81).
set cycle to 0.

wait until mach > MachForZoom.
clearscreen.
print "Mode: Zoom Climb" at (0,1).
until targetp > PitchForZoom {
  wait 0.4.
  set targetp to targetp + 0.2.
  if TWR < 0.6 {                    // Evalua por falta de aire
    set cycle to 1.
    ag1 on.
  }
}

if cycle = 0 {      // Si los motores seguían airbreathing, cambiar
  wait until TWR < 0.6.
  set cycle to 1.
  ag1 on.
}

until targetp < PitchForMECO {
  wait 0.6.
  set targetp to targetp - 0.2.
}

// MECO

lock throttle to throt.
wait until ship:apoapsis > 68000.
print "Llegue" at (0,2).
set difference to targetapoapsis-ship:apoapsis.
until ship:apoapsis > targetapoapsis {
  set throt to max(0.05,(targetapoapsis-ship:apoapsis)/difference).
  print "Throttle: "+round(throt,2) at (0,1).
}
set throt to 0.

//COAST

clearscreen.
print "Mode: Coasting" at (0,1).
lock steering to srfprograde.

// CIRCULARIZE

wait until eta:apoapsis < 60.
sas off.
clearscreen.
print "Mode: Standing by for Circularization Burn" at (0,1).
lock steering to heading(90,0).
lock difv to 2294-apospeed().

wait until eta:apoapsis < fburntime(difv)*CirculariBurnFactor.
clearscreen.
print "Mode: Circularizing" at (0,1).
set throt to 1.
until ship:periapsis>70085  {
	if verticalspeed < 0 {
	set throt to min(1,1.05-ship:periapsis/70000).
	}
	else {
	set corr to eta:apoapsis-fburntime(difv)*CirculariBurnFactor.
	set throt to min(1,max(0,throt-corr*0.05)).
	}
}
set throt to 0.

// DEPLOY SISTEMS AND FINISH

sas off.
rcs off.
clearscreen.
print "Orbit achieved".
ag2 on.
wait 7.
ag3 on.
unlock steering.
SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.
