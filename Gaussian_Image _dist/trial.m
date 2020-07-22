imagepath="./train"
Samples=[]
for k=1:19
    I=imread(sprintf('%s/%03d.png',imagepath,k));
    R=I(:,:,1);
    G=I(:,:,2);
    B=I(:,:,3);
    disp('click the ball and cover its perimeter and whencome to starting point double click this will create a mask')
    figure(1),Mask=roipoly(I);
    figure(2),imshow(Mask),title('Mask1');
    sample_ind=find(Mask>0);
    R=R(sample_ind)
    G=G(sample_ind)
    B=B(sample_ind)
    Samples=[Samples;[R G B]]
    
    
    disp('Press any key to continue and ctrl+c to exit')
    pause
    
end
% visualize the sample distribution
figure(3),scatter3(Samples(:,1),Samples(:,2),Samples(:,3))
xlabel('Red');
ylabel('green');
zlabel('blue')
