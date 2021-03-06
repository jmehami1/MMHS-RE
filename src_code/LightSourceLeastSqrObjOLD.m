function [f,j] = LightSourceLeastSqrObj(in1,E,in3,in4,in5,in6)
%LIGHTSOURCELEASTSQROBJ
%    [F,J] = LIGHTSOURCELEASTSQROBJ(IN1,E,IN3,IN4,IN5,IN6)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    31-Jan-2021 13:24:50

phi_0 = in1(1);
mu = in1(2);
r_d = in1(3);

n1 = in4(1,:)';
n2 = in4(2,:)';
n3 = in4(3,:)';

s1 = in5(1);
s2 = in5(2);
s3 = in5(3);

v1 = in6(1);
v2 = in6(2);
v3 = in6(3);


x1 = in3(1,:)';
x2 = in3(2,:)';
x3 = in3(3,:)';

t2 = n1.*s1;
t3 = n2.*s2;
t4 = n3.*s3;
t5 = n1.*x1;
t6 = n2.*x2;
t7 = n3.*x3;
t8 = s1.*v1;
t9 = s2.*v2;
t10 = s3.*v3;
t11 = v1.*x1;
t12 = v2.*x2;
t13 = v3.*x3;
t14 = r_d.^2;
t15 = s1.^2;
t16 = s2.^2;
t17 = s3.^2;
t18 = x1.^2;
t19 = x2.^2;
t20 = x3.^2;
t21 = s1.*x1.*2.0;
t22 = s2.*x2.*2.0;
t23 = s3.*x3.*2.0;
t24 = -x1;
t25 = -x2;
t26 = -x3;
t36 = r_d.*s1.*x1.*4.0;
t37 = r_d.*s2.*x2.*4.0;
t38 = r_d.*s3.*x3.*4.0;
t39 = mu./2.0;
t27 = -t5;
t28 = -t6;
t29 = -t7;
t30 = -t8;
t31 = -t9;
t32 = -t10;
t33 = -t21;
t34 = -t22;
t35 = -t23;
t40 = r_d.*t15.*2.0;
t41 = r_d.*t16.*2.0;
t42 = r_d.*t17.*2.0;
t43 = r_d.*t18.*2.0;
t44 = r_d.*t19.*2.0;
t45 = r_d.*t20.*2.0;
t46 = s1+t24;
t47 = s2+t25;
t48 = s3+t26;
t49 = -t36;
t50 = -t37;
t51 = -t38;
t52 = -t39;
t53 = t46.^2;
t54 = t47.^2;
t55 = t48.^2;
t56 = t11+t12+t13+t30+t31+t32;
t58 = t2+t3+t4+t27+t28+t29;
t63 = t15+t16+t17+t18+t19+t20+t33+t34+t35;
t57 = t53+t54+t55;
t59 = t56.^mu;
t64 = t63.^t52;
t60 = sqrt(t57);
t61 = t60.^3;
t62 = t14.*t60;
t65 = t40+t41+t42+t43+t44+t45+t49+t50+t51+t61+t62;
t66 = 1.0./t65;
f = E-phi_0.*t14.*t58.*t59.*t64.*t66;

j = [-t14.*t58.*t59.*t64.*t66,-phi_0.*t14.*t58.*t59.*t64.*t66.*(log(t56)-log(t63)./2.0),phi_0.*r_d.*t58.*t59.*t64.*t66.*-2.0+phi_0.*t14.*t58.*t59.*t64.*t66.^2.*(t15.*2.0+t16.*2.0+t17.*2.0+t18.*2.0+t19.*2.0+t20.*2.0+r_d.*t60.*2.0-s1.*x1.*4.0-s2.*x2.*4.0-s3.*x3.*4.0)];
end
