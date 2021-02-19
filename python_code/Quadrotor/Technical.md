# Rotor Position

\*Global means : Quadrotor in Global Coordinates

+phi = between y+ to z+
+theta = between x+ to z-
+psi = between y+ to x+

## Rotor 1 -> direction = +x (left)
rotorPosition = quadrotor position + rotationMatrix*[armLength, 0, 0]

## Rotor 2 -> direction = -x (right)
rotorPosition = quadrotor position + rotationMatrix*[-armLength, 0, 0

## Rotor 3 -> direction = +y (Front)
rotorPosition = quadrotor position + rotationMatrix*
[0, armLength, 0]

## Rotor 4 -> direction = -y (behind)
rotorPosition = quadrotor position + rotationMatrix*
[0, -armLength, 0]
