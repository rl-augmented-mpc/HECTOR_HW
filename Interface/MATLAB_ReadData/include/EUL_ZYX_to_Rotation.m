function R = EUL_ZYX_to_Rotation(eul)
Rx = rotx(eul(1));
Ry = roty(eul(2));
Rz = rotz(eul(3));

% R = (Rx*Ry*Rz);
R = (Rz*Ry*Rx);