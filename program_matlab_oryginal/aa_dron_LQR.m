% (C) F.A.Dul

clear

global az_turbulence;
global ax_wind;
global az_wind;

%n = 6;  % dimension of x
n = 8;  % With engines
m = 2;  % dimension of u

z0 = 2.0;
h_flight = 1.0;  % Over the terrain
c_turb = 1000.0;
X_turb_1 = 1500.0;
X_turb_2 = 2000.0;

az_turbulence = 0.0;
ax_wind = 0.0;
az_wind = 0.0;

rad2deg = 180/pi;
deg2rad = pi/180;


if n == 6 ,
 x = [ 0.0; 0.0; 0.0; 0.0; -z0; 0.0 ];
end
if n == 8 ,
 x = [ 0.0; 0.0; 0.0; 0.0; -z0; 0.0; 0.0; 0.0 ];
end

u = [ 0.0 ; 0.0 ];

Vel = 0.10; % / 3.6;

dt = 0.01;
t  = 0.0;
for i = 1 : 10000 ,

  if( t >= 100.0 )
    break;
  end
  
  X = x(4);

  Z0 = 5.0;
  [ z_terr , alfa ] = aa_trajectory( X , Vel , dt );
  Vx = Vel*cos(alfa);
  Vz = Vel*sin(alfa);

  z_ref = z_terr + h_flight;
 
  tp(i)   = t;
  yp(:,i) = x;
  up(:,i) = u;
  gp(i)   = z_terr;
  zp(i)   = z_ref;

  x_ref = X;

  R = eye(m,m);
  Q = eye(n,n);
  Q(1,1) = 1000.0;      
  Q(2,2) = 1000.0;      
  Q(3,3) = 0.1;        
  Q(4,4) = 10.0;    
  Q(5,5) = 100.0;  
  Q(6,6) = 1.0e+03;    
  
  if( n == 8 )
    Q = 10000*Q; 
  end  

  e = zeros(n,1);
  e(1) = x(1) - ( cos(x(6))*Vx + sin(x(6))*Vz );
  e(2) = x(2) - ( sin(x(6))*Vx - cos(x(6))*Vz );
  e(3) = x(3) - 0;
  e(4) = x(4) - (x_ref);
  e(4) = 0.0;
  e(5) = x(5) - (-z_ref);
  e(6) = x(6) - 0.0;

  [A,B] = aa_matrices_AB( "aa_rhs" , x , t , u , n , m );
  [K,P] = lqr_m( A , B , Q , R );
  u = -K * e;
  
  umax = 10000.0;
  u(1) = max( -umax , min( umax , u(1) ) ); 
  u(2) = max( -umax , min( umax , u(2) ) ); 

  az_turbulence = 0.0;
  ax_wind = 0.0;
  az_wind = 0.0;
  if X > X_turb_1 && X < X_turb_2 ,
    az_turbulence = c_turb*( 1.0 - 2.0*rand() );
    ax_wind = 0.0;
    az_wind = 0.0;  %15.5 + 3.0*( 1.0 - 2.0*rand() );
  end

  %x = aa_rk45( "aa_rhs" , x , t , dt , u );
  x = aa_euler( "aa_rhs" , x , t , dt , u );

  if  mod( i , 20 ) == 0 , 
    refresh;

    xl = 0; %max( 0 , X - 1 );
    xp = 5.0;
    zl = 0.0;
    zu = 8.0;

    v_x = Vx; %sin(x(6))*x(1) - cos(x(6))*x(2);
    v_z = Vz; %sin(x(6))*x(1) - cos(x(6))*x(2);
    V = sqrt( x(1)^2 + x(2)^2 );
    e_v = Vel - V;
    teta = x(6)*rad2deg;
    alt = -x(5);
    T1 = u(1);
    T2 = u(2);
    gamma = atan( ( sin(x(6))*x(1) - cos(x(6))*x(2))  / ( cos(x(6))*x(1) + sin(x(6))*x(2)) )*rad2deg;

    [ xs , zs ] = aa_mdl( x(4) , -x(5) , x(6) , 0.50 );

    figure( 1 , 'position' , [ 300 , 100 , 1100 , 640 ] );
    txt=sprintf('t=%7.3f V=%10.5f m/s  v_x=%10.5f v_z=%10.5f  teta=%10.5f  alfa=%10.5f |||  u1=%7.4f  u2=%7.4f   |||   e_v=%10.5f  e(z)=%10.5f', t , V , v_x , v_z , teta , alfa*rad2deg , T1 , T2 , e_v , e(5) );
    set( 0 , 'defaulttextfontsize' , 12 );
    plot( yp(4,:) , zp(:) , 'r' , yp(4,:) , gp(:) , 'g' , yp(4,:) , -yp(5,:) , 'b' , yp(4,i) , -yp(5,i) , 'bo' ,
          xs(1:5) , zs(1:5) , 'k' , "linewidth" , 3
        );
    set( 0 , 'defaulttextfontsize' , 12 );
    %plot( yp(4,:) , 100+50*up(2,:) , 'g' , yp(4,:) , 100*up(1,:) , 'g' , yp(4,:) , -gp(:) , 'r' , yp(4,:) , -yp(5,:) , 'b' , yp(4,i) , -yp(5,i) , 'bo' , 'linewidth' , 2 );
    title(txt);
    axis( [ xl, xp, zl , zu ] );
  end

  t = t + dt;

end

pause




