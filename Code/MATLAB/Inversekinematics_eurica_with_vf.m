%% Inversekinematics_test
clc
clf

hold on
grid on
axis equal

%vf=pi/2;

x_in=0.140;
y_in=-0.090;
z_in=0.140;

x=x_in;
y=y_in;
z=z_in;

y_0=y;

P=[x z y];

%n=46;
n=100;
i=1;
r=0.05;

if(y > 0)
    display("ERROR! POSITIVE Y-VALUE!")
end

takeStep=1;

k=1;

for i=1:1
    
    if(n > 1 && takeStep)
        k=k+1;
        pause(.01)
        %x=r*cos(pi/20 * k);
        x=x-0.005;
        %z=r*sin(pi/20 * i);
        y=y_0+0.05*sin(pi*k/20);
    end
    
    if(n > 1 && ~takeStep)
        k=k+1;
        pause(.01)
        x=x+0.005;
        %z=r*sin(pi/20 * i);
    end
    
    if(k > 18)
        takeStep=~takeStep;
        k=1;
    end
    
    clf
    L1=0.095;
    L2=0.140;
    
    plot3(x,z,y,'or')

    coxa=0.047;
    
%     x_p=z*cos(vf);
%     z_p=z*sin(vf);
%     
%     x=z+x_p;
%     z=z+z_p;

    L=sqrt(x^2+z^2);
    HF=sqrt((L-coxa)^2+y^2);
    A1=atan((L-coxa)/-y);
    %A1=pi/2-A1;
    %A1=A1-pi/2;
    %A1=-A1;
    
    A2=acos((L2^2-L1^2-HF^2)/(-2*L1*HF));
    %A2=pi/2-A2;
    %A2=-A2; % Denna tillsammans med "A1=pi/2-A1" fick det att fungera för z=0
    
    B1=acos((HF^2-L1^2-L2^2)/(-2*L1*L2));
    %B1=pi/2-A2;
    
    v1=-(pi/2-A1-A2);
    %v1=A1+A2-pi/2;
    B2=pi-v1-B1;
    v2=pi/2-B1;
    v0=atan2(z,x);

    %v0 =  v0 - pi/4;
    
    P_0=[0,0,0];

    x0=cos(v0)*coxa;
    y0=0;
    z0=sin(v0)*coxa;
    P0=[x0,z0,y0];
    
    x1=cos(v0)*(coxa+L1*cos(v1));
    y1=sin(v1)*L1;
    z1=sin(v0)*(coxa+L1*cos(v1));

    P1=[x1,z1,y1];
    
    x2=x1+cos(v0)*L2*cos(B2);
    y2=y1-L2*sin(B2);
    %z2=z1+cos(v0)*sin(v0)*L2*cos(B2);
    z2=z1+sin(v0)*L2*cos(B2);

    P2=[x2 z2 y2];
    
    pts=[P0;P1;P2];
    pts2=[P_0;P0];

    xline(0)
    yline(0)

    line(pts2(:,1),pts2(:,2))
    line(pts(:,1),pts(:,2),pts(:,3))

    xlabel('x')
    ylabel('z')
    zlabel('y')

    view(3)
    
    xlim([-0.2 0.2])
    ylim([-0.2 0.2])
    zlim([-0.2 0.2])
    
end

display("L1 = "+norm(P1-P0))
display("L2 = "+norm(P2-P1))

display("HF = "+norm(P2-P0)+", ALSO: HF = "+HF)
display("0 to P2: "+norm(P-P0))

if(norm(P1-P0) ~= L1 || norm(P2-P1) ~= L2)
    display("Impossible!")
else
    display("GÜDD! L1 = L1, L2 = L2")
end

subspace(P2',P')*180/pi
subspace(P2',P')*180/pi

legend({"A1 ="+A1*180/pi,"A2 ="+A2*180/pi,"B1 ="+B1*180/pi,"B2 ="+B2*180/pi,"v1 ="+v1*180/pi,"v2 ="+v2*180/pi},'Location','southeast')
v0*180/3.14
v1*180/3.14
v2*180/3.14


