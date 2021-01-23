# Rotor Position

\*Global means : Quadrotor in Global Coordinates

## Rotor 1

[x,y,z] = [xGlobal + Cos(theta)*armLength , yGlobal, zGLobal - Sin(theta)*armLength]

## Rotor 2

[x,y,z] = [xGlobal - Cos(theta)*almLength, yGlobal, zGlobal + Sin(theta)*armLength]

## Rotor 3

[x,y,z] = [xGlobal , yGlobal + Cos(phi)*armLength, zGlobal+ Sin(phi)*armLength]

## Rotor 4

[x,y,z] = [xGlobal, yGlobal - Cos(phi)*armLength, zGlobal - Sin(phi)*armLength]
