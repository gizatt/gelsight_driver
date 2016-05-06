% Generate lookup table from ball video, and using the lookup table to
% reconstruct the 3D image

CaliFile='Cali_Aly1_Flat.mat';

name1='Cali_Aly1_Flat';
name2='_s';
border=50;
avrobj=VideoReader([name1 name2 '.mp4']);
load(CaliFile);
%% Ini on the first frame

f=read(avrobj, 1);
thresh=30;
[f0, f00]=iniFrame(f, border);
f01=sum(f0,3);
size1=size(f0,1);size2=size(f0,2);

frame_=f(border+1:end-border,border+1:end-border,:);
I=double(frame_);
dI=mean(f0-I,3);


%% 3D reconstruct
Frn=240;
frame=read(avrobj, Frn);
frame_=frame(border+1:end-border,border+1:end-border,:);
I=double(frame_)-f0;

[ImGradX, ImGradY]=matchGrad(LookupTable, I, f0,f01);
figure,imagesc(sqrt(ImGradX.^2+ImGradY.^2));
heightmap=fast_poisson2(ImGradX, ImGradY);
figure,mesh(heightmap);