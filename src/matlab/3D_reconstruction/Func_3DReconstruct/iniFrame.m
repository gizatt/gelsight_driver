function [f0, f1]=iniFrame(frame0, border)
% frame0 is the raw data from camera
kscale=50;
kernnel=fspecial('gaussian',[kscale*2 kscale*2],kscale*1);
frame0_=double(frame0);
f0(:,:,1)=conv2(frame0_(:,:,1),kernnel,'same');
f0(:,:,2)=conv2(frame0_(:,:,2),kernnel,'same');
f0(:,:,3)=conv2(frame0_(:,:,3),kernnel,'same');
f0=f0(border+1:end-border,border+1:end-border,:);
f1=f0;


frame_=frame0_(border+1:end-border,border+1:end-border,:);
dI=mean(f0-frame_,3);
id=find(dI<5);
pixcount=size(f0,1)*size(f0,2);
f0(id)=f0(id)*0.15+frame_(id)*0.85;
f0(id+pixcount)=f0(id+pixcount)*0.15+frame_(id+pixcount)*0.85;
f0(id+pixcount*2)=f0(id+pixcount*2)*0.15+frame_(id+pixcount*2)*0.85;



