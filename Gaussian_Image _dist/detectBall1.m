% Robotics: Estimation and Learning 
% WEEK 1
% 
% Complete this function following the instruction. 
%function [segI, loc] = detectBall(I)
% function [segI, loc] = detectBall(I)
%
% INPUT
% I       120x160x3 numerial array 
%
% OUTPUT
% segI    120x160 numeric array
% loc     1x2 or 2x1 numeric array 
imagepath="./train"
k=1
am=imread(sprintf('%s/%03d.png',imagepath,k));
pim=[];
pim1=[];
imshow(am);
im=double(am);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hard code your learned model parameters here
%
MR=mean(R)
MG=mean(G)
MB=mean(B)
SR=std(double(R))
SG=std(double(G))
SB=std(double(B))
 mu = [MR MG MB];
 pim=[];
 pim1=[];
 
%mu =  [149.7198  144.5757   60.8763];
%sig = diag([180.8987  128.4632  339.5755]);
 sig = [SR*SR 0 0 ;0 SG*SG 0 ;0 0 SB*SB ];
 %sig1 = cov(double(Samples));
 %sig=[sig1(1,1) 0 0; 0 sig1(2,2) 0; 0 0 sig1(3,3)]
 %sig = [SR 0 0 ;0 SG 0 ;0 0 SB ];
 thres = 0.0000001;
 [n m s]=size(im);
 mask1=zeros(n,m);
 bw_biggest=false(n,m);
%mask1=int(mask1)
 for i=1:n
    pim=[]
    for j=1:m
     %a= im(i,j,:)-mu;
    %fprintf('print','%.4f',a);
    %scale=double(1/(2*pi)^((length(sig(:,1)))/2)*(det(sig))^0.5);
    %S=size(im(i,j,:));
    %imrgb=reshape(im(i,j,:),[S(1)*S(2),S(3)]);
    %e=exp(-0.5*(imrgb'-mu')'*sig^(-1)*(imrgb'-mu'));
    %p=scale*e;
    p=mvnpdf(double([im(i,j,1) im(i,j,2) im(i,j,3)]),mu,sig)
   
    if p > thres
        mask1(i,j)=1;
 
    end
 pim=[pim  p]   
 end
 pim1=[pim1 ; pim]
 end
CC=bwconncomp(mask1);
%CC = bwconncomp(bw);

numPixels = cellfun(@numel,CC.PixelIdxList);
[biggest,idx] = max(numPixels);
bw_biggest(CC.PixelIdxList{idx}) = true; 
figure,
imshow(bw_biggest); hold on;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Find ball-color pixels using your model
% % http://www.mathworks.com/help/images/ref/regionprops.html
S = regionprops(CC,'Centroid');
loc = S(idx).Centroid;
plot(loc(1), loc(2),'r+');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Do more processing to segment out the right cluster of pixels.
% You may use the following functions.
%   bwconncomp
%   regionprops
% Please see example_bw.m if you need an example code.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute the location of the ball center
%

% segI = 
% loc = 
% 
% Note: In this assigment, the center of the segmented ball area will be considered for grading. 
% (You don't need to consider the whole ball shape if the ball is occluded.)

%end
