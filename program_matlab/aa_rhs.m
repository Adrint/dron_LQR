function [ dx_dt ] = aa_rhs( x , t , u )

  global az_turbulence;
  global ax_wind;
  global az_wind;

  n = size(x)(1);
  dx_dt = zeros( n , 1 );

  deg2rad = pi/180.0;
  g = 9.81;

  S = 1.0;
  
  mass = 25.0;
  Iy = 100;

  vx = x(1) + ax_wind;
  vz = x(2) + az_wind;

  alpha = atan2( vz , vx );
  V = sqrt( vz*vz + vx*vx );

  CD_0 = 0.30;
  CD = CD_0;

  ro_0 = 1.225;
  ro = ro_0 * ( 1.0 - abs(x(5))/44300.0 )^4.256;

  Q_dyn = 0.5*ro*V*V;

  L = 0.0;
  D = Q_dyn * S * CD;
  G = mass * g;

  Th = 1.0;
  %Thrust_1 = Th * u(1);
  %Thrust_2 = Th * u(2);
  Thrust_1 = 0.5*G + u(1);
  Thrust_2 = 0.5*G + u(2);
  %Thrust_1 = 0.5*G + x(7);
  %Thrust_2 = 0.5*G + x(8);
  if( n == 8 )
    Thrust_1 = x(7);
    Thrust_2 = x(8);
  end  

  cm_q = -0.01;
  
  Tau = 0.05;

  beta = 0.0*deg2rad;
  cb = cos(beta);
  sb = sin(beta);
  
  dx_dt(1) = ( -D*cos(alpha) + L*sin(alpha) - G*sin(x(6)) - Thrust_1*sb + Thrust_2*sb ) / mass  -  x(3)*vz;
  dx_dt(2) = ( -D*sin(alpha) - L*cos(alpha) + G*cos(x(6)) - Thrust_1*cb - Thrust_2*cb ) / mass  +  x(3)*vx  + az_turbulence;
  dx_dt(3) = ( 0.5*( Thrust_2*cb - Thrust_1*cb ) + cm_q*x(3) ) / Iy;
  dx_dt(4) =  cos(x(6))*vx + sin(x(6))*vz;
  dx_dt(5) = -sin(x(6))*vx + cos(x(6))*vz;
  dx_dt(6) = x(3);
  
  if( n == 8 )
    dx_dt(7) = (1.0/Tau)*( -x(7) + Th*u(1) );
    dx_dt(8) = (1.0/Tau)*( -x(8) + Th*u(2) );
  end

endfunction
