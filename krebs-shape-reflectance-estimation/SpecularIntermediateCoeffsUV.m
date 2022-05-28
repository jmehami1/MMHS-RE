function coefs = SpecularIntermediateCoeffsUV(s_u,s_v,m_u,m_v,d_u,d_v,zeta_uv,zeta_u,zeta_v)
%SPECULARINTERMEDIATECOEFFSUV
%    COEFS = SPECULARINTERMEDIATECOEFFSUV(S_U,S_V,M_U,M_V,D_U,D_V,ZETA_UV,ZETA_U,ZETA_V)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    23-Aug-2021 11:51:28

t2 = d_u.*2.0;
t3 = d_v.*2.0;
t4 = m_u.*2.0;
t5 = m_v.*2.0;
t8 = s_u.*zeta_uv.*2.0;
t9 = s_v.*zeta_uv.*2.0;
t6 = t4.*zeta_uv;
t7 = t5.*zeta_uv;
t14 = t8.*zeta_u;
t15 = t8.*zeta_v;
t16 = t9.*zeta_u;
t17 = t9.*zeta_v;
t10 = t6.*zeta_u;
t11 = t6.*zeta_v;
t12 = t7.*zeta_u;
t13 = t7.*zeta_v;
t20 = t14.*zeta_v;
t21 = t16.*zeta_v;
t18 = t10.*zeta_v;
t19 = t12.*zeta_v;
coefs = [2.0,2.0,-2.0,t2-t3-t4+t5+t6-t8+t9+t12+t13+t14+t15+t18+t21-m_v.*zeta_uv.*2.0-m_u.*zeta_u.*zeta_uv.*2.0-m_u.*zeta_v.*zeta_uv.*2.0-s_v.*zeta_u.*zeta_uv.*2.0-s_v.*zeta_v.*zeta_uv.*2.0-m_v.*zeta_u.*zeta_v.*zeta_uv.*2.0-s_u.*zeta_u.*zeta_v.*zeta_uv.*2.0,-t2+t3+t4-t5+t7+t8-t9+t10+t11+t16+t17+t19+t20-m_u.*zeta_uv.*2.0-m_v.*zeta_u.*zeta_uv.*2.0-m_v.*zeta_v.*zeta_uv.*2.0-s_u.*zeta_u.*zeta_uv.*2.0-s_u.*zeta_v.*zeta_uv.*2.0-m_u.*zeta_u.*zeta_v.*zeta_uv.*2.0-s_v.*zeta_u.*zeta_v.*zeta_uv.*2.0];
