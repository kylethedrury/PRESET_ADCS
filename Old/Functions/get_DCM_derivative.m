% THIS IS A SCRIPT FOR GETTING THE DCM GIVEN A QUATERNION q

function output = get_DCM(q)

    DCM11 = q(1)^2 - 