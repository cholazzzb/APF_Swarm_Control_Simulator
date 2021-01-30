# Rotor Position

\*Global means : Quadrotor in Global Coordinates

+phi = between y+ to z+
+theta = between x+ to z-
+psi = between y+ to x+

## Rotor 1 -> direction = +x

[x,y,z] = [xGlobal + Cos(theta)*armLength + Cos(psi)*armLength, yGlobal - Sin(psi) *armLength, zGLobal - Sin(theta)*armLength]

## Rotor 2 -> direction = -x

[x,y,z] = [xGlobal - Cos(theta)*almLength - Cos(psi)*armLength, yGlobal + Sin(psi)*armLength, zGlobal + Sin(theta)*armLength]

## Rotor 3 -> direction = +y

[x,y,z] = [xGlobal + Sin(psi)*armLength, yGlobal + Cos(phi)*armLength + Cos(psi)*armLength, zGlobal+ Sin(phi)*armLength]

## Rotor 4 -> direction = -y

[x,y,z] = [xGlobal - Sin(psi)*armLength, yGlobal - Cos(phi)*armLength - Cos(psi)*armLength, zGlobal - Sin(phi)*armLength]
