function K = control_FURPEN(A,B,p1,p2,p3,p4)

    % Desired poles (-30 and -40 are given)
    poles = [p1, p2, p3, p4];
    % Find control gain using MATLAB pole-placement command (acker or place)
    K = acker(A, B, poles);

end