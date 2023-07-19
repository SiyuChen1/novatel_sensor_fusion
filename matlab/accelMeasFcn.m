function z = accelMeasFcn(obj, x)
    % used in the insfilterasync
    % https://www.mathworks.com/help/nav/ref/insfilterasync.html
    q0   = x(1);  
    q1   = x(2);  
    q2   = x(3);  
    q3   = x(4);
    % q = q0 + q1*i + q2 * j + q3 * k
    % rotation matrix from NED to accelerometer, i.e. IMU frame

    wx   = x(5); %#ok<NASGU>
    wy   = x(6); %#ok<NASGU>
    wz   = x(7); %#ok<NASGU>
    % gyroscope measurements in the IMU frame

    pn   = x(8); %#ok<NASGU>
    pe   = x(9); %#ok<NASGU>
    pd  = x(10); %#ok<NASGU>
    % position in NED

    vn  = x(11); %#ok<NASGU>
    ve  = x(12); %#ok<NASGU>
    vd  = x(13); %#ok<NASGU>
    % velocity in NED

    an  = x(14);
    ae  = x(15); 
    ad  = x(16); 
    % acceleration in NED

    accx_b  = x(17); 
    accy_b  = x(18); 
    accz_b  = x(19); 
    % acceleration bias in accelerometer, i.e IMU frame

    gyrox_b  = x(20); %#ok<NASGU>
    gyroy_b  = x(21); %#ok<NASGU>
    gyroz_b  = x(22); %#ok<NASGU>
    % gyro bias in IMU frame

    magNavX  = x(23); %#ok<NASGU>
    magNavY  = x(24); %#ok<NASGU>
    magNavZ  = x(25); %#ok<NASGU>
    magX  = x(26); %#ok<NASGU>
    magY  = x(27); %#ok<NASGU>
    magZ  = x(28); %#ok<NASGU>

    rf = obj.ReferenceFrameObject;
    grav = zeros(1,3, 'like', x);
    grav(rf.GravityIndex) = rf.GravitySign*rf.GravityAxisSign*gravms2();
    gnavx = grav(1);
    gnavy = grav(2);
    gnavz = grav(3);
    
        z = [...    
            accx_b - (an - gnavx)*(q0^2 + q1^2 - q2^2 - q3^2) + (ad - gnavz)*(2*q0*q2 - 2*q1*q3) - (ae - gnavy)*(2*q0*q3 + 2*q1*q2)
            accy_b - (ae - gnavy)*(q0^2 - q1^2 + q2^2 - q3^2) - (ad - gnavz)*(2*q0*q1 + 2*q2*q3) + (an - gnavx)*(2*q0*q3 - 2*q1*q2)
            accz_b - (ad - gnavz)*(q0^2 - q1^2 - q2^2 + q3^2) + (ae - gnavy)*(2*q0*q1 - 2*q2*q3) - (an - gnavx)*(2*q0*q2 + 2*q1*q3)];
    
    % z: acceleration measurements in IMU frame
    % acc_b: acceleration bias in IMU frame
    % R: rotation matrix from NED to IMU frame
    % a_g: gravity vector expressed in NED
    % a_{NED}: actual acceleration expressed in NED
    % z = acc_b + R^{-1} (a_g - a_{NED})

    % question: why a_g - a_{NED} instead of a_g + a_{NED}
    % according to https://www.oxts.com/de/accelerometers/
    % when object is stationary, accelerometer measurement = g
    % when object falls free, accelerometer measurement = 0
    % so a_g -a_{NED} is correct  

 end