declare parameter XXX.

// Funciones utilizadas:

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

// Variables ajustables de prueba:

set deorbitperiapsis to 35000.
set deorbitburnLNG to 0.


// Configuracion inicial:

SAS off.
RCS off.
Lights off.
lock throttle to 0.
gear off.
panels on.
set KSCwest to latlng( -0.048597000539, -74.72335052490). 
set KSCeast to latlng( -0.05028, -74.48821).
lock currentpos to SHIP:GEOPOSITION.

clearscreen.

EVALUAR SI LA ORBITA TIENE QUE SER CIRCULARIZADA
	CIRCULARIZAR


when alt:radar < 10000 then {rcs off. print "RCS OFF".}

SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.	


