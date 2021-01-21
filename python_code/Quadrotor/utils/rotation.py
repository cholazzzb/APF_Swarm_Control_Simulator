import math

def BodyToInertiaFrame(angles):
    phi = angles[0]
    theta = angles[0]
    psi = angles[0]
    return [[math.cos(psi)*math.cos(theta), math.sin(psi) * math.cos(theta), -math.sin(theta)],
            [-math.sin(psi)*math.cos(phi)+math.cos(psi)*math.sin(theta)*math.sin(phi), math.cos(psi) *
             math.cos(phi) + math.sin(psi)*math.sin(theta)*math.sin(phi), math.cos(theta)*math.sin(phi)],
            [math.sin(psi)*math.sin(phi)+math.cos(psi)*math.sin(theta)*math.cos(phi), -math.cos(psi) * math.sin(phi)+math.sin(psi)*math.sin(theta)*math.cos(phi), math.cos(theta)*math.cos(phi)]]

def InertiaToBodyFrame(angles):
    phi = angles[0]
    theta = angles[0]
    psi = angles[0]
    return [[math.cos(psi)*math.cos(theta), -math.sin(psi)*math.cos(phi)+math.cos(psi)*math.sin(theta)*math.sin(phi), math.sin(psi)*math.sin(phi)+math.cos(psi)*math.sin(theta)*math.cos(phi)],
            [math.sin(psi)*math.cos(theta), math.cos(psi)*math.cos(phi)+math.sin(psi)*math.sin(theta)
             * math.sin(phi), -math.cos(psi)*math.sin(phi)+math.sin(psi)*math.sin(theta)*math.cos(phi)],
            [-math.sin(theta), math.cos(theta)*math.sin(phi), math.cos(theta)*math.cos(phi)]]

