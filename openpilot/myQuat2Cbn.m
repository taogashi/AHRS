function y=myQuat2Cbn(quat)
quat = reshape(quat,1,4);
y=quat2dcm(quat)';
end