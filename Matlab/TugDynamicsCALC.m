function [M,C,D,Dn,eta_dot] = TugDynamicsCALC(in1)
%TugDynamicsCALC
%    [M,C,D,Dn,ETA_DOT] = TugDynamicsCALC(IN1)

%    This function was generated by the Symbolic Math Toolbox version 9.0.
%    06-Jun-2022 18:36:42

%in1=states=[xn;yn;psi;u;v;r]
psi = in1(3,:);
r = in1(6,:);
u = in1(4,:);
v = in1(5,:);
M = reshape([8.708580283640001e+5,0.0,0.0,0.0,1.180858028364e+6,-5.0e+5,0.0,1.0e+6,7.960656982913189e+7],[3,3]);
if nargout > 1
    t2 = cos(psi);
    t3 = abs(u);
    t4 = sin(psi);
    t5 = r+v;
    t6 = r.*2.0;
    t7 = r.*3.0;
    t8 = r.*4.0;
    t9 = r.*5.0;
    t10 = r.*6.0;
    t11 = r.*7.0;
    t13 = r.*8.0;
    t14 = r.*9.0;
    t15 = r.*1.0e+1;
    t16 = r.*1.1e+1;
    t17 = r.*1.2e+1;
    t18 = r.*1.3e+1;
    t19 = r.*1.4e+1;
    t20 = r.*1.5e+1;
    t21 = r.*1.6e+1;
    t22 = -v;
    t29 = r./2.0;
    t30 = r.*(3.0./2.0);
    t31 = r.*(5.0./2.0);
    t32 = r.*(7.0./2.0);
    t49 = r.*(9.0./2.0);
    t50 = r.*(1.1e+1./2.0);
    t51 = r.*(1.3e+1./2.0);
    t52 = r.*(1.5e+1./2.0);
    t53 = r.*(1.7e+1./2.0);
    t54 = r.*(1.9e+1./2.0);
    t55 = r.*(2.1e+1./2.0);
    t56 = r.*(2.3e+1./2.0);
    t57 = r.*(2.5e+1./2.0);
    t58 = r.*(2.7e+1./2.0);
    t59 = r.*(2.9e+1./2.0);
    t60 = r.*(3.1e+1./2.0);
    t61 = r.*(3.3e+1./2.0);
    t98 = r.*2.5e+5;
    t99 = u.*8.0e+4;
    t100 = v.*3.9e+5;
    t171 = r.*7.908580283640001e+5;
    C = reshape([0.0,t171,t5.*2.5e+5+v.*1.4e+5,-t171,0.0,-t99,t5.*-2.5e+5-v.*1.4e+5,t99,0.0],[3,3]);
end
if nargout > 2
    D = reshape([1.741716056728e+4,0.0,0.0,0.0,1.476072535455e+4,0.0,0.0,0.0,7.960656982913189e+6],[3,3]);
end
if nargout > 3
    t12 = abs(t5);
    t23 = r+t5;
    t24 = t5+t6;
    t25 = t5+t7;
    t26 = t5+t8;
    t27 = t5+t9;
    t28 = t5+t10;
    t39 = r+t22;
    t40 = t5+t11;
    t41 = t5+t13;
    t42 = t5+t14;
    t43 = t5+t15;
    t44 = t5+t16;
    t45 = t5+t17;
    t46 = t5+t18;
    t47 = t5+t19;
    t48 = t5+t20;
    t72 = t29+v;
    t73 = t30+v;
    t74 = t31+v;
    t75 = t32+v;
    t76 = t6+t22;
    t77 = t7+t22;
    t78 = t8+t22;
    t79 = t9+t22;
    t80 = t10+t22;
    t81 = t11+t22;
    t86 = t49+v;
    t87 = t50+v;
    t88 = t51+v;
    t89 = t52+v;
    t90 = t53+v;
    t91 = t54+v;
    t92 = t55+v;
    t93 = t56+v;
    t94 = t57+v;
    t95 = t58+v;
    t96 = t59+v;
    t97 = t60+v;
    t101 = t13+t22;
    t102 = t14+t22;
    t103 = t15+t22;
    t104 = t16+t22;
    t105 = t17+t22;
    t106 = t18+t22;
    t107 = t19+t22;
    t108 = t20+t22;
    t109 = t21+t22;
    t128 = t22+t29;
    t129 = t22+t30;
    t130 = t22+t31;
    t131 = t22+t32;
    t141 = t22+t49;
    t142 = t22+t50;
    t143 = t22+t51;
    t144 = t22+t52;
    t145 = t22+t53;
    t146 = t22+t54;
    t147 = t22+t55;
    t148 = t22+t56;
    t149 = t22+t57;
    t150 = t22+t58;
    t151 = t22+t59;
    t152 = t22+t60;
    t153 = t22+t61;
    t33 = abs(t23);
    t34 = abs(t24);
    t35 = abs(t25);
    t36 = abs(t26);
    t37 = abs(t27);
    t38 = abs(t28);
    t62 = abs(t39);
    t63 = abs(t40);
    t64 = abs(t41);
    t65 = abs(t42);
    t66 = abs(t43);
    t67 = abs(t44);
    t68 = abs(t45);
    t69 = abs(t46);
    t70 = abs(t47);
    t71 = abs(t48);
    t82 = abs(t72);
    t83 = abs(t73);
    t84 = abs(t74);
    t85 = abs(t75);
    t110 = abs(t86);
    t111 = abs(t87);
    t112 = abs(t88);
    t113 = abs(t89);
    t114 = abs(t90);
    t115 = abs(t91);
    t116 = abs(t92);
    t117 = abs(t93);
    t118 = abs(t94);
    t119 = abs(t95);
    t120 = abs(t96);
    t121 = abs(t97);
    t122 = abs(t76);
    t123 = abs(t77);
    t124 = abs(t78);
    t125 = abs(t79);
    t126 = abs(t80);
    t127 = abs(t81);
    t132 = abs(t101);
    t133 = abs(t102);
    t134 = abs(t103);
    t135 = abs(t104);
    t136 = abs(t105);
    t137 = abs(t106);
    t138 = abs(t107);
    t139 = abs(t108);
    t140 = abs(t109);
    t154 = abs(t128);
    t155 = abs(t129);
    t156 = abs(t130);
    t157 = abs(t131);
    t158 = abs(t141);
    t159 = abs(t142);
    t160 = abs(t143);
    t161 = abs(t144);
    t162 = abs(t145);
    t163 = abs(t146);
    t164 = abs(t147);
    t165 = abs(t148);
    t166 = abs(t149);
    t167 = abs(t150);
    t168 = abs(t151);
    t169 = abs(t152);
    t170 = abs(t153);
    t172 = t5.*t12.*8.45807816688347e+2;
    t173 = t39.*t62.*8.465623268214189e+2;
    et1 = t172-t173+t23.*t33.*8.395202322460815e+2+t24.*t34.*8.332326478038158e+2+t25.*t35.*8.271965667392404e+2+t26.*t36.*8.213281545931261e+2+t27.*t37.*8.154597424470116e+2;
    et2 = t28.*t38.*8.095913303008969e+2+t40.*t63.*8.037229181547824e+2+t41.*t64.*7.978545060086678e+2+t42.*t65.*7.919860938625533e+2+t43.*t66.*7.861176817164386e+2;
    et3 = t44.*t67.*7.80249269570324e+2+t45.*t68.*7.514940500543627e+2+t46.*t69.*6.54665249643472e+2+t47.*t70.*4.698985305341956e+2+t48.*t71.*1.339488636363636e+2+t72.*t82.*8.4895160890948e+2;
    et4 = t73.*t83.*8.426640244672141e+2+t74.*t84.*8.363764400249485e+2+t75.*t85.*8.30130772812298e+2+t86.*t110.*8.242623606661837e+2-t76.*t122.*8.59990057011688e+2;
    et5 = t87.*t111.*8.183939485200688e+2-t77.*t123.*8.920520561890112e+2+t88.*t112.*8.125255363739543e+2-t78.*t124.*9.596983741971574e+2+t89.*t113.*8.066571242278396e+2;
    et6 = t79.*t125.*(-1.197861800195613e+3)+t90.*t114.*8.007887120817251e+2-t80.*t126.*1.316802303365308e+3+t91.*t115.*7.949202999356104e+2-t81.*t127.*1.434187534665856e+3;
    et7 = t92.*t116.*7.890518877894958e+2+t93.*t117.*7.831834756433814e+2+t94.*t118.*7.735005956022923e+2+t95.*t119.*7.294875045064329e+2+t96.*t120.*5.711120769883375e+2;
    et8 = t97.*t121.*3.175972325143879e+2-t101.*t132.*1.555466180493116e+3-t102.*t133.*1.668055935415801e+3-t103.*t134.*1.024669058444421e+3-t104.*t135.*2.422999305369258e+2-t105.*t136.*1.695679012345679e+2-t106.*t137.*1.400833333333333e+2;
    et9 = t107.*t138.*(-1.173055555555556e+2)-t108.*t139.*9.452777777777778e+1-t109.*t140.*(2.87e+2./4.0)-t128.*t154.*8.552391933517455e+2-t129.*t155.*8.524607153688843e+2-t130.*t156.*8.725006982779683e+2-t131.*t157.*9.194225148788333e+2;
    et10 = t141.*t158.*(-1.125403801710639e+3)-t142.*t159.*1.26286188264802e+3-t143.*t160.*1.373548211752225e+3-t144.*t161.*1.494826857579486e+3-t145.*t162.*1.613081611022401e+3;
    et11 = t146.*t163.*(-1.425862089866242e+3)-t147.*t164.*4.25372953619864e+2-t148.*t165.*2.118916867355559e+2-t149.*t166.*1.514722222222222e+2-t150.*t167.*1.286944444444444e+2-t151.*t168.*1.059166666666667e+2-t152.*t169.*8.313888888888889e+1-t153.*t170.*6.036111111111111e+1+v.*abs(v).*8.520954011306128e+2;
    et12 = t172+t173+t23.*t33.*1.679040464492163e+3+t24.*t34.*2.499697943411447e+3+t25.*t35.*3.308786266956962e+3+t26.*t36.*4.10664077296563e+3+t27.*t37.*4.89275845468207e+3+t28.*t38.*5.667139312106278e+3;
    et13 = t40.*t63.*6.429783345238259e+3+t41.*t64.*7.180690554078011e+3+t42.*t65.*7.919860938625533e+3+t43.*t66.*8.647294498880825e+3+t44.*t67.*9.362991234843888e+3+t45.*t68.*9.769422650706714e+3;
    et14 = t46.*t69.*9.165313495008608e+3+t47.*t70.*7.048477958012933e+3+t48.*t71.*2.143181818181818e+3+t72.*t82.*4.2447580445474e+2+t73.*t83.*1.263996036700821e+3+t74.*t84.*2.090941100062371e+3;
    et15 = t75.*t85.*2.905457704843043e+3+t86.*t110.*3.709180622997827e+3+t76.*t122.*1.719980114023376e+3+t87.*t111.*4.501166716860378e+3+t77.*t123.*2.676156168567033e+3+t88.*t112.*5.281415986430703e+3;
    et16 = t78.*t124.*3.838793496788629e+3+t89.*t113.*6.049928431708797e+3+t79.*t125.*5.989309000978063e+3+t90.*t114.*6.806704052694663e+3+t80.*t126.*7.900813820191846e+3+t91.*t115.*7.551742849388299e+3;
    et17 = t81.*t127.*1.003931274266099e+4+t92.*t116.*8.285044821789706e+3+t93.*t117.*9.006609969898886e+3+t94.*t118.*9.668757445028654e+3+t95.*t119.*9.848081310836844e+3+t96.*t120.*8.281125116330895e+3;
    et18 = t97.*t121.*4.922757103973012e+3+t101.*t132.*1.244372944394493e+4+t102.*t133.*1.501250341874221e+4+t103.*t134.*1.024669058444421e+4+t104.*t135.*2.665299235906184e+3+t105.*t136.*2.034814814814815e+3+t106.*t137.*1.821083333333333e+3+t107.*t138.*1.642277777777778e+3;
    et19 = t108.*t139.*1.417916666666667e+3+t109.*t140.*1.148e+3+t128.*t154.*4.276195966758727e+2+t129.*t155.*1.278691073053326e+3+t130.*t156.*2.181251745694921e+3+t131.*t157.*3.217978802075917e+3+t141.*t158.*5.064317107697876e+3;
    et20 = t142.*t159.*6.945740354564111e+3+t143.*t160.*8.928063376389464e+3+t144.*t161.*1.121120143184614e+4+t145.*t162.*1.371119369369041e+4+t146.*t163.*1.35456898537293e+4;
    et21 = t147.*t164.*4.466416013008572e+3+t148.*t165.*2.436754397458893e+3+t149.*t166.*1.893402777777778e+3+t150.*t167.*1.737375e+3+t151.*t168.*1.535791666666667e+3+t152.*t169.*1.288652777777778e+3+t153.*t170.*9.959583333333333e+2;
    Dn = [t3.*u.*(3.228524982009454e+4./((log(t3.*3.35e+7)./log(1.0e+1)-2.0).^2+1.0./1.0e+3)+8.609399952025211e+3);et1+et2+et3+et4+et5+et6+et7+et8+et9+et10+et11;et12+et13+et14+et15+et16+et17+et18+et19+et20+et21];
end
if nargout > 4
    eta_dot = [t4.*t22+t2.*u;t4.*u+t2.*v;r];
end