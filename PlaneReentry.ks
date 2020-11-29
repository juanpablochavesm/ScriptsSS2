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

FUNCTION timeToAltitude {
    // Returns time in seconds to the next time SHIP crosses the input altitude or 0 if
	//    input altitude is never crossed
    // Usage: timeToAltitude(<altitude in meters>)
	// written by /u/only_to_downvote

	PARAMETER alt.

	// return 0 if never reach altitude
    IF alt < SHIP:PERIAPSIS OR alt > SHIP:APOAPSIS RETURN 0.

    // query constants
	LOCAL ecc IS SHIP:OBT:ECCENTRICITY.
    IF ecc = 0 SET ecc TO 0.00001. // ensure no divide by 0
	LOCAL sma IS SHIP:OBT:SEMIMAJORAXIS.
    LOCAL desiredRadius IS alt + SHIP:BODY:RADIUS.
	LOCAL currentRadius IS SHIP:ALTITUDE + SHIP:BODY:RADIUS.

	// Step 1: get true anomaly (bounds required for numerical errors near apsides)
    LOCAL desiredTrueAnomalyCos IS MAX(-1, MIN(1, ((sma * (1-ecc^2) / desiredRadius) - 1) / ecc)).
	LOCAL currentTrueAnomalyCos IS MAX(-1, MIN(1, ((sma * (1-ecc^2) / currentRadius) - 1) / ecc)).

    // Step 2: calculate eccentric anomaly
	LOCAL desiredEccentricAnomaly IS ARCCOS((ecc+desiredTrueAnomalyCos) / (1 + ecc*desiredTrueAnomalyCos)).
    LOCAL currentEccentricAnomaly IS ARCCOS((ecc+currentTrueAnomalyCos) / (1 + ecc*currentTrueAnomalyCos)).

    // Step 3: calculate mean anomaly
	LOCAL desiredMeanAnomaly IS desiredEccentricAnomaly - ecc  * SIN(desiredEccentricAnomaly).
    LOCAL currentMeanAnomaly IS currentEccentricAnomaly - ecc  * SIN(currentEccentricAnomaly).
	IF ETA:APOAPSIS > ETA:PERIAPSIS
    {
	    SET currentMeanAnomaly TO 360 - currentMeanAnomaly.
    }.
	IF alt < SHIP:ALTITUDE
    {
	    SET desiredMeanAnomaly TO 360 - desiredMeanAnomaly.
    }
	ELSE IF alt > SHIP:ALTITUDE AND ETA:APOAPSIS > ETA:PERIAPSIS
    {
	    SET desiredMeanAnomaly TO 360 + desiredMeanAnomaly.
    }.

    // Step 4: calculate time difference via mean motion
	LOCAL meanMotion IS 360 / SHIP:OBT:PERIOD. // in deg/s
    RETURN (desiredMeanAnomaly - currentMeanAnomaly) / meanMotion.
}

FUNCTION advanced_heading {
    	PARAMETER myHeading,myPitch,myRoll.
    	LOCAL returnDir IS HEADING(myHeading,myPitch).
    	RETURN ANGLEAXIS(myRoll,returnDir:FOREVECTOR) * returnDir.
    }

FUNCTION RadarAltitude {
    	return ship:altitude - ship:geoposition:terrainheight.
    }

// Variables ajustables de prueba:

set KPpitchpid to 0.1.
set KIpitchpid to 0.
set KDpitchpid to 0.
set KProllpid to 0.01.
set KIrollpid to 0.003.
set KDrollpid to 0.005.

// Configuracion inicial:

if true {
  set SteeringManager:ROLLCONTROLANGLERANGE to 110.
  set hea to ship:facing.
  rcs off.
  lights off.
  sas off.
  set longimpact to 0.
  set latimpact to 0.
  set throt to 0.
  lock throttle to throt.
  set KSCwest to latlng( -0.048597000539, -74.72335052490).
  set KSCeast to latlng( -0.05028, -74.48821).
  lock currentpos to SHIP:GEOPOSITION.
  set pitchpid to pidloop(KPpitchpid,KIpitchpid,KDpitchpid,-1,1).
  set rollpid to pidloop(KProllpid,KIrollpid,KDrollpid,-1,1).
  set firstrun6 to true.
  set firstrun5 to true.
  set firstrun4 to true.
  set firstrun8 to true.
  set firstrun7 to true.

}

// VECTORES Y ANGULOS

if true {
  lock rightrotation to ship:facing*r(0,90,0).
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
}

// PROGRAMA PRINCIPAL

set runmode to 1.

until runmode = 0 {

  if Abort {
    SET SHIP:CONTROL:NEUTRALIZE TO TRUE.
    unlock steering.
    unlock throttle.
    clearscreen.
    print "Program Aborted.".
    break.
  }                             // Abort con devolución de controles

  if runmode = 1 {                          // Circularización si es necesaria
    print "Runmode 1: Orbit Circularization" at (0,1).
    if ship:Periapsis > 70000 {              // Corrobora estar en órbita
      if ship:apoapsis < 74000 {              // Evalúa si ya es circular
        set runmode to 2.
        clearscreen.                   // Si ya es circular, pasa a retroburn
      }
      else {                                  // Sino, hay que circularizar
        if ship:periapsis < 71000 {            // Si el periapsis ya es bajo
          panels on.
          wait until eta:periapsis < 60.
          set warp to 0.
          wait 30.
          sas on.
          wait 2.
          set sasmode to "retrograde".
          lock DeltaV to perispeed()-2290.
          wait until eta:periapsis < fburntime(DeltaV).
          set throt to 0.2.
          until ship:apoapsis < 71000  {
          	if verticalspeed > 0 {
          	set throt to 0.2.
          	}
          	else {
          	set corr to eta:periapsis-fburntime(DeltaV)*0.5.
          	set throt to min(1,max(0,throt-corr*0.05)).
          	}
          }
          set throt to 0.
          set runmode to 2.
          clearscreen.
        }
        else {                                 // Sino, hay que bajar el periapsis y el apoapsis
          wait until eta:apoapsis < 60.           // Bajar periapsis
          set warp to 0.
          wait 30.
          sas on.
          wait 2.
          set sasmode to "retrograde".
          set TargetSpeed to (body:mu*(2/(ship:apoapsis+body:radius)-2/(71000 + ship:apoapsis+body:radius*2)))^0.5.
          lock DeltaV to apospeed()-TargetSpeed.
          wait until eta:apoapsis < fburntime(DeltaV).
          set throt to 0.5.
          until ship:periapsis < 70500  {
          	if verticalspeed < 0 {
          	set throt to 0.1.
          	}
          	else {
          	set corr to eta:apoapsis-fburntime(DeltaV).
          	set throt to min(1,max(0,throt-corr*0.05)).
          	}
          }
          set throt to 0.                         // Periapsis quedó bajo
          wait until eta:periapsis < 60.           // Bajar apoapsis
          set warp to 0.
          wait 30.
          sas on.
          wait 2.
          set sasmode to "retrograde".
          lock DeltaV to perispeed()-2290.
          wait until eta:periapsis < fburntime(DeltaV).
          set throt to 0.2.
          until ship:apoapsis < 71000  {
          	if verticalspeed > 0 {
          	set throt to 0.1.
          	}
          	else {
          	set corr to eta:apoapsis-fburntime(DeltaV).
          	set throt to min(1,max(0,throt-corr*0.05)).
          	}
          }
          set throt to 0.                         // Circularizado
          set runmode to 2.
          clearscreen.
        }
      }
    }
    else {                                   // Si estaba suborbital, pasa a 3
      clearscreen.
      set runmode to 3.
    }
  }

  if runmode = 2 {                          // Retroburn de reentrada
    print "Runmode 2: Reentry Burn" at (0,1).
    if currentpos:lng > 133 {
      if currentpos:lng < 135 {
        set warp to 0.
        wait 28.
        unlock steering.
        sas on.
        wait 2.
        set sasmode to "retrograde".
        wait 16.
        until ship:periapsis < 36000 {
          set throt to 0.2.
        }
        set throt to 0.
        set runmode to 3.
        wait 5.
        lock steering to hea.
        clearscreen.
      }
    }
  }

  if runmode = 3 {                          // Preparar sistemas para reentrada

    if ship:altitude < 70000 {             // Si ya está en atmósfera, pasa a 4
      set runmode to 4.
      clearscreen.
      sas off.
    }
    else {
      print "Runmode 3: Preparing for Atmospheric contact." at (0,1).
      SET STEERINGMANAGER:SHOWFACINGVECTORS TO false.
      sas off.
      panels off.
      wait 4.
      ag2 off.
      set targetp to 30.
      lock steering to hea.
      set hea to lookdirup(heading(90,targetp):vector,up:vector).
      set runmode to 301.
      clearscreen.
      sas off.
    }

  }

  if runmode = 301 {                        // Coast hasta reentrada
    print "Runmode 301: Coasting to Atmospheric contact." at (0,1).
    set SteeringManager:MAXSTOPPINGTIME to 3.
    SET STEERINGMANAGER:SHOWFACINGVECTORS TO false.
    lock steering to hea.
    set hea to lookdirup(heading(90,targetp):vector,up:vector).
    if ship:altitude < 70000 {
      set runmode to 4.
      clearscreen.
    }
  }

  if runmode = 4 {                      // Reentrada hasta periapsis menor a cero
    if firstrun4 {
      set firstrun4 to false.
      set targetp to 30.
      set PreviousDifference to 0.
      lock steering to hea.
    }
    if periapsis < 70 {                   // Pasar a 5 cuando llegue a cero
      set runmode to 5.
      clearscreen.
    }
    else if ship:altitude < 64000 {
      print "Runmode 4: Suborbital periapsis decrease." at (0,1).
      set SteeringManager:MAXSTOPPINGTIME to 3.
      lock x to ship:altitude/1000.
      set targetperiapsis4 to (-0.27*x^2+37.5*x-1261.4)*1000.
      set peridifference to targetperiapsis4 - ship:periapsis.
      print "Ship Periapsis " at (0,3). print round(ship:periapsis) at (23,3).
      print "Target Periapsis: " at (0,4). print round(targetperiapsis4) at (23,4).
      print "Periapsis Difference: " at (0,5). print round(peridifference) at (23,5).                         // DEBUGGING
      set targetrate to -peridifference*0.02.         // RATE: TASA A LA CUAL CAMBIA PERIDIFFERENCE
      print "Target Rate: " at (0,7). print round(targetrate,1) at (23,7).
      set shiprate to peridifference - Previousdifference.
      set ratedifference to shiprate - targetrate.
      print "Ship Rate: " at (0,8). print round(shiprate,1) at (23,8).
      print "Rate Difference: " at (0,9). print round(ratedifference,1) at (23,9).
      set targetp to max(pitchangle-1,min(pitchangle+1,targetp - ratedifference*0.03)).    // MECANISMO DE CONTROL POR ALTITUD-PERIAPSIS
      print "Target Pitch: " at (0,11). print round(targetp) at (23,11).                          // DEBUGGING
      print "Pitch: " at (0,12). print  round(pitchangle) at (23,12).
      set hea to lookdirup(heading(90,targetp):vector,up:vector).
      set PreviousDifference to peridifference.
      wait 0.1.
    }
    else {
      print "Runmode 4: Suborbital periapsis decrease." at (0,1).
      set SteeringManager:MAXSTOPPINGTIME to 3.
      set targetp to 30.
      set hea to lookdirup(heading(90,targetp):vector,up:vector).
    }
  }

  if runmode = 5 {                      // Reentrada

    if firstrun5 {
      set firstrun5 to false.
      set targetp to 26.
      set PreviousDifference to 0.
      lock steering to hea.
    }
    print "Runmode 5: Reentry attitude control." at (0,1).
    set SteeringManager:MAXSTOPPINGTIME to 3.
    set longimpact to ship:body:geopositionof(POSITIONAT(SHIP,time:seconds+timeToAltitude(70))):lng.
    set latimpact to ship:body:geopositionof(POSITIONAT(SHIP,time:seconds+timeToAltitude(70))):lat.
    print "Impact longitude: " at (0,3). print round(longimpact) at (23,3).
    lock targetlong to -0.0001048*ship:geoposition:lng^3-0.0051429*ship:geoposition:lng^2+1.8416217*ship:geoposition:lng+51.9360360.
    if ship:geoposition:lng > -90 {
      set targetlong to -78.7.
    }
    print "Target longitude: " at (0,4). print  round(targetlong) at (23,4).                             // MECANISMO DE CONTROL POR ALTITUD-LONGITUD DE IMPACTO
    set longdifference to targetlong-longimpact.
    print "Longitudes Difference: " at (0,5). print  round(longdifference,1) at (23,5).
    set targetrate to -longdifference*0.04.
    print "Target Rate: " at (0,7). print  round(targetrate,2) at (23,7).
    set shiprate to longdifference - PreviousDifference.
    print "Ship Rate: " at (0,8). print  round(shiprate,2) at (23,8).
    set ratedifference to shiprate - targetrate.
    print "Rate Difference: " at (0,9). print  round(ratedifference,2) at (23,9).
    set targetp to max(max(pitchangle-1,0),min(pitchangle+1,targetp - ratedifference*5)).
    print "Target pitch: " at (0,11). print  round(targetp) at (23,11).
    print "Impact latitude: " at (0,14). print  round(latimpact) at (23,14).
    set targetroll to max(-2,min(2,-latimpact)).
    print "Target roll: " at (0,15). print  round(targetroll) at (23,15).
    set hea to advanced_heading(90,targetp,targetroll).
    print "Pitch: " at (0,12). print  round(pitchangle) at (23,12).
    set PreviousDifference to longdifference.
    wait 0.1.
    if ship:altitude < 25000 {
      set runmode to 501.
      clearscreen.
    }
  }

  if runmode = 501 {
    print "Runmode 501: Pre-Transition to aerodynamic flight." at (0,1).
    set Distance to ((ship:geoposition:position-KSCwest:position):mag)/1000.
    if airspeed < 300+Distance {
      set runmode to 6.
      clearscreen.
    }
    if ship:altitude < 15000 {
      set previousaltitude to altitude.
      set Distance to ((ship:geoposition:position-KSCwest:position):mag)/1000.
      set previousdistance to Distance.
      set runmode to 7.
    }
  }

  if runmode = 6 {                    // Transición a vuelo aerodinamico
    if firstrun6 {
      set firstrun6 to false.
      set previousroll to rollangle.
      set previouspitch to pitchangle.
    }
    print "Runmode 6: Transition to aerodynamic flight." at (0,1).
    if ship:altitude < 15000 and vang(up:vector,ship:srfprograde:vector) < 118 {
      set runmode to 7.
      set previousaltitude to altitude.
      set Distance to ((ship:geoposition:position-KSCwest:position):mag)/1000.
      set previousdistance to Distance.
      clearscreen.
    }
    else {
      unlock steering.
      //  print "Pitch: " + round(pitchangle) at (0,2).
      set targetp to pitchangle-(absaoa-14).
      set targetr to 90-vang(forehor,north:vector).
      set targetPrate to (targetp - pitchangle)*0.03.       // POSITIVO SI HAY QUE SUBIR
      set targetRrate to (targetr - rollangle)*0.03.       // POSITIVO SI HAY QUE GIRAR CLOCKWISE
      set pitchrate to pitchangle - previouspitch.        // POSITIVO SI SUBIENDO
      set rollrate to rollangle - previousroll.          //  POSITIVO SI GIRANDO CLOCKWISE
      set pitchratediff to targetPrate - pitchrate.
      set rollratediff to targetRrate - rollrate.
      // set ship:control:pitch to min(1,max(-1,ship:control:pitch + pitchpid:update(time:seconds,pitchratediff))).
      // set ship:control:roll to max(-1,min(1,ship:control:roll + rollpid:update(time:seconds,rollratediff))).
      set ship:control:pitch to 3*pitchratediff.
      set ship:control:roll to max(-1,min(1,0.05*rollratediff)).
      print "Target pitch: " at (0,3). print  round(targetp,1) at (24,3).
      print "Target pitch rate: " at (0,4). print  round(targetprate,2) at (24,4).
      print "Effective Pitch rate: " at (0,5). print  round(pitchrate,2) at (24,5).
      print "Control stick Pitch: " at (0,6). print  round(ship:control:pitch,1) at (24,6).
      print "Target Roll: " at (0,8). print  round(targetr,1) at (24,8).
      print "Target Roll rate: " at (0,9). print  round(targetrrate,2) at (24,9).
      print "Effective Roll rate: " at (0,10). print  round(rollrate,2) at (24,10).
      print "Control stick Roll: " at (0,11). print  round(ship:control:roll,1) at (24,11).
      wait 0.05.
      set previouspitch to pitchangle.
      set previousroll to rollangle.
    }
  }

  if runmode = 7 {                    // Acercamiento y alineación con la pista
    print "Runmode 7: Unpowered glide and alignment." at (0,1).
    set ship:control:yaw to 0.01.
    set Distance to ((ship:geoposition:position-KSCwest:position):mag)/1000.
    set Energia to (80*(airspeed)^0.89-0.00006*airspeed^3 + ship:altitude-70 - 3770)/1000.
    // print "Ship total energy: " at (0,2). print round(Energia,1) at (24,2).
    //print "Distance to Runway: " at (0,3). print round(Distance,1) at (24,3).
    set Qe to Energia/Distance.
    //print "Coefficient Q: " at (0,4). print round(Qe,2) at (24,4).
    set shiplat to ship:geoposition:lat.
    set aimlat to KSCwest:lat + (KSCwest:lat - shiplat)*2.
    set aimpos to latlng(aimlat,KSCwest:lng).
    set targetr to (vang(lefthor,aimpos:altitudeposition(ship:altitude))-90).
    if Qe > 0.41 {
      set runmode to 8.
      set ship:control:yaw to 0.01.
      set firstrun8 to true.
      //clearscreen.
    }
    set targetaltitude to Distance*180-100.
    set targetvertical to (Distance-previousdistance)*180 + (targetaltitude-altitude)*0.005.
    set vertical to altitude-previousaltitude.
    if firstrun7 {
      set timer to time.
      set firstrun7 to false.
    }
    if time > timer + 3 {
      set timer to time.
      set targetp to max(-20,min(0,targetp + (targetvertical - vertical))).
      print round(airspeed) + "   " + round(altitude) + "   " + round(Distance).
      // print "Pitch corrected by " + (targetvertical - vertical) at (0,7).
    }
    print "Glideslope altitude:" at (0,2). print round(targetaltitude,3) at (24,2).
    print "Targeted vertical:" at (0,3). print round(targetvertical,3) at (24,3).
    print "Current vertical:" at (0,4). print round(vertical,3) at (24,4).
    print "Target roll:" at (0,5). print round(targetr,3) at (24,5).

    set targetRrate to max(-1,min(1,(targetr - rollangle)*0.2)).       // POSITIVO SI HAY QUE GIRAR CLOCKWISE
    set rollrate to rollangle - previousroll.          //  POSITIVO SI GIRANDO CLOCKWISE
    set rollratediff to targetRrate - rollrate.
    set ship:control:roll to max(-1,min(1,0.3*rollratediff)).
    set ship:control:yaw to 0.01.
    set targetPrate to (targetp - pitchangle)*0.03.       // POSITIVO SI HAY QUE SUBIR
    set pitchrate to pitchangle - previouspitch.        // POSITIVO SI SUBIENDO
    set pitchratediff to targetPrate - pitchrate.
    set ship:control:pitch to 0.5*pitchratediff.
    set previouspitch to pitchangle.
    set previousroll to rollangle.
    set previousaltitude to altitude.
    set previousdistance to Distance.
    print "Target pitch:" at (0,6). print round(targetp) at (24,6).
  }

  if runmode = 8 {                    // Energy bleed by banking
    print "Runmode 8: Energy management." at (0,1).
    set ship:control:yaw to 0.01.
    set Distance to ((ship:geoposition:position-KSCwest:position):mag)/1000.
    set Energia to (80*(airspeed)^0.89-0.00006*airspeed^3 + ship:altitude-70 - 3770)/1000.
    print "Ship total energy: " at (0,2). print round(Energia,1) at (24,2).
    print "Distance to Runway: " at (0,3). print round(Distance,1) at (24,3).
    set Qe to Energia/Distance.
    print "Coefficient Q: " at (0,4). print round(Qe,2) at (24,4).
    set shiplat to ship:geoposition:lat.
    set aimlat to KSCwest:lat + (KSCwest:lat - shiplat)*2.
    set aimpos to latlng(aimlat,KSCwest:lng).
    set targetrx to (vang(lefthor,aimpos:altitudeposition(ship:altitude))-90).
    if firstrun8 {
      set targetr to targetrx+35.
    }
    if vang(lefthor,aimpos:altitudeposition(ship:altitude)) < 85 {
      set targetr to targetrx-35.
    }
    if vang(lefthor,aimpos:altitudeposition(ship:altitude)) > 95 {
      set targetr to targetrx+35.
    }
    if Qe < 0.4 {
      set runmode to 7.
      set previousaltitude to altitude.
      set previousdistance to Distance.
      // clearscreen.
    }
    if Distance < 20 {
      set runmode to 7.
      set previousaltitude to altitude.
      set previousdistance to Distance.
      // clearscreen.
    }
    print "Target roll:" at (0,5). print round(targetr) at (24,5).
    set targetRrate to max(-1,min(1,(targetr - rollangle)*0.2)).       // POSITIVO SI HAY QUE GIRAR CLOCKWISE
    set rollrate to rollangle - previousroll.          //  POSITIVO SI GIRANDO CLOCKWISE
    set rollratediff to targetRrate - rollrate.
    set ship:control:roll to max(-1,min(1,0.3*rollratediff)).
    set ship:control:pitch to 0.3.
    set ship:control:yaw to 0.01.
    set previouspitch to pitchangle.
    set previousroll to rollangle.
    if time > timer + 1 {
      set timer to time.
      print round(airspeed) + "   " + round(altitude) + "   " + round(Distance).
    }
  }

  if runmode = 9 {                    // Acercamiento final y aterrizaje
    print "Runmode 9: Final Approach." at (0,1).

  }

  if runmode = 1000 {                 // Aterrizaje de emergencia donde se pueda
    print "Abort Mode 1: Contingency landing. Press Abort to override." at (0,1).
  }

  if runmode = 1100 {               // Recuperación en caso de pérdida de control en vuelo
    print "Abort Mode 2: Loss of Vehicle Control. Press Abort to override." at (0,1).
  }

}

Print "Program finished." at (0,2).
