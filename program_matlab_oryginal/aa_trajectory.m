function [ Z , alpha ] = aa_trajectory( X , Vel , dt )

  %Z = 10.0*sin( 1.5*X ) + 3.0*sin( 2.11*X ) + 1.0*sin( 3.43*X ) + 1.0*sin( 4.7*X );
  %dx = Vel*dt;
  %X1 = X + dx;
  %Z1 = 10.0*sin( 1.5*X1 ) + 3.0*sin( 2.11*X1 ) + 1.0*sin( 3.43*X1 ) + 1.0*sin( 4.7*X1 );
  %alpha = atan2( Z1 - Z , dx );

  Z = 1.0;
  if( X <= 1.0 )
    Z = 1.0;
  end  
  if( X > 1.0 && X < 1.5 )
    Z = 1.0 + (X-1.0)*10.0;
  end  
  if( X >= 1.5 && X <= 2.0 )
    Z = 6.0;
  end  
  if( X >= 2.0 && X <= 2.5 )
    Z = 6.0 - (X-2.0)*10.0;
  end  
  if( X >= 2.5 )
    Z = 1.0;
  end  
    
  Z1 = Z;

  dx = Vel*dt;
  X = X + dx;
  if( X <= 1.0 )
    Z = 1.0;
  end  
  if( X > 1.0 && X < 1.5 )
    Z = 1.0 + (X-1.0)*10.0;
  end  
  if( X >= 1.5 && X <= 2.0 )
    Z = 6.0;
  end  
  if( X >= 2.0 && X <= 2.5 )
    Z = 6.0 - (X-2.0)*10.0;
  end  
  if( X >= 2.5 )
    Z = 1.0;
  end  
  
  alpha = atan2( Z - Z1 , dx );
  
  
endfunction
