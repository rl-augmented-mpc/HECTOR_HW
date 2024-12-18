function R = EUL_ZYX_to_R_wb(eul)
Rx = rotx(eul(1));
Ry = roty(eul(2));
Rz = rotz(eul(3));

R = (Rz*Ry*Rx)';