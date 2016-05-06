function [ImGradX, ImGradY]=matchGrad(LookupTable, dI, f0,f01, validmask)
% LookupTable is the look up table structure; dI is the difference; 
% f0 is the initializaion image, f01 is the local weight. In current
% sketch, it's the sum of three channels. 
% validmask is the mask for contact area, optional

size1=size(dI,1);size2=size(dI,2);
sizet=size1*size2;sizet2=2*sizet;
ImGradX=zeros(size1,size2);
ImGradY=zeros(size1,size2);
dI=dI./f0;
binm=LookupTable.bins-1;
if exist('validmask')
    validid=find(validmask);
    r1=dI(validid)./f01(validid);
    g1=dI(validid+sizet)./f01(validid);
    b1=dI(validid+sizet2)./f01(validid);
    r2=(r1-LookupTable.Zeropoint)/LookupTable.Scale;r2=fix1(r2);
    g2=(g1-LookupTable.Zeropoint)/LookupTable.Scale;g2=fix1(g2);
    b2=(b1-LookupTable.Zeropoint)/LookupTable.Scale;b2=fix1(b2);
    r3=floor(r2*binm)+1;b3=floor(b2*binm)+1;g3=1+floor(g2*binm);
    ind=sub2ind([LookupTable.bins LookupTable.bins LookupTable.bins], r3,g3,b3);
    ImGradX(validid)=LookupTable.GradX(ind);
    ImGradY(validid)=LookupTable.GradY(ind);
    
else    % when there is no mask and all are calculated
    r1=dI(:,:,1)./f01;g1=dI(:,:,2)./f01;b1=dI(:,:,3)./f01;
    r2=(r1-LookupTable.Zeropoint)/LookupTable.Scale;r2=fix1(r2);
    g2=(g1-LookupTable.Zeropoint)/LookupTable.Scale;g2=fix1(g2);
    b2=(b1-LookupTable.Zeropoint)/LookupTable.Scale;b2=fix1(b2);
    r3=floor(r2*binm)+1;b3=floor(b2*binm)+1;g3=1+floor(g2*binm);
    ind=sub2ind([LookupTable.bins LookupTable.bins LookupTable.bins], r3,g3,b3);
    ImGradX=LookupTable.GradX(ind);
    ImGradY=LookupTable.GradY(ind);
end
    

function num=fix1(num)
% fix between 0 to 1
num(num>1)=1;num(num<0)=0;