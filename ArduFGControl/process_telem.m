
clear;

f = csvread('telemetry.csv');
%fid = fopen("telemetry.csv");
%f = fscanf(fid,'%f,%f,%f',[3, 1000]);
f = f';


len =size(f,2)

s = 50;
l = round(len*0.9);
range = [s:l];

t = [range]/20;
r2d = 180/pi;

long = f(1,range);
lat = f(2,range);
alt = f(3,range);

p = f(4,range)*r2d;
q = f(5,range)*r2d;
r = f(6,range)*r2d;

err_head = f(12,range);
derr_head = f(13,range);
ierr_head = f(14,range);

% control surfaces
elev = f(7,range);
rud = f(8,range);
ail = f(9,range);

dq = f(10,range);
iq = f(11,range);

epitch  = f(15,range);
dpitch = f(16,range);
ipitch = f(17,range);

tpitch = f(18,range);
speed = f(19,range);

tdx = f(20,range);
tdy = f(21,range);

wind = f(22,range);
wind_dir = f(23,range);
press = f(24,range);

dr = f(25,range);
ir = f(26,range);

pitch = f(27,range);

accel_x = f(28,range);
accel_y = f(29,range);
accel_z = f(30,range);


y = (lat-lat(end))*6e6;
y2 = (long-long(end))*6e6.*cos(lat);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if (0)
figure(1);
%instead of lat(end) should use coords of actual target, and make an x where the target is
subplot(2,1,1),plot(t,y);
ylabel('dist feet');
xlabel('time seconds');
subplot(2,1,2),plot(t, y2);
ylabel('dist feet');
xlabel('time seconds');
end;

figure(10);
subplot(2,1,1),
%plot(y2,y,'b');
plot(tdx,tdy,'b');
hold on;
plot(0,0,'rx');
hold off;
xlabel('distance feet');
ylabel('distance feet');
subplot(2,1,2),plot(sqrt(tdx.*tdx+tdy.*tdy),alt,'b');
xlabel('distance');
ylabel('altitude');

figure(2),
subplot(2,1,1),plot(t,alt);
ylabel('altitude feet');
xlabel('time seconds');

subplot(2,1,2),plot(t,speed);
ylabel('feet/second');
legend('speed');

figure(3),
subplot(2,1,1),plot(t,p, t,q,t,r);
legend('p','q','r');
ylabel('radians/second');
subplot(2,1,2),plot(t,rud, t,elev,t,ail);
legend('rudder','elevator','aileron');

if (0)
figure(6),
subplot(3,1,1),plot(t,wind);
ylabel('wind knots');
subplot(3,1,2),plot(t,wind_dir);
ylabel('wind dir degrees');
subplot(3,1,3),plot(t,press);
ylabel('pressure inhg');
end

figure(5),
plot(t,rud, t,err_head, t, derr_head*10, t, ierr_head/10);
ylabel('degrees');
legend('rudder', 'error heading', 'derror heading', 'ierr head');

figure(4),
factor = 100; % max(abs(q))/ max(abs(elev));
subplot(2,1,1),plot(t,elev*factor, t, q,t,dq*500, t, iq/10);
ylabel('radians/sec, or other units');
legend('elevator','q', 'dq','iq');
subplot(2,1,2),plot(t, elev, t,pitch, t, tpitch, t, dpitch*50, t, ipitch/5,t,epitch);
legend('elev', 'gps derived pitch', 'tpitch', 'dpitch', 'ipitch', 'error pitch');


if (0)
figure(7);
plot(t, r, t,dr,t,ir);
legend('r','dr','ir');

end;

figure(20)
plot(t,accel_x,t,accel_y, t,accel_z);
legend('x','y','z');

