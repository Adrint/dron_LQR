function [ xs , ys ] = aa_mdl( X , Y , teta , c )

  xs0 = [ -0.1*c , +0.1*c , +0.1*c , -0.1*c , -0.1*c ];
  ys0 = [ -0.1*c , -0.1*c , +0.1*c , +0.1*c , -0.1*c ];

  for i = 1 : 5 ,
    xs(i) = X + xs0(i)*cos(teta) - ys0(i)*sin(teta);
    ys(i) = Y + xs0(i)*sin(teta) + ys0(i)*cos(teta);
  end

endfunction
