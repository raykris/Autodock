function [M,C,D,Dn] = ContactFriction(in1)
%ContactFriction
%    [M,C,D,Dn] = ContactFriction(IN1)

%    This function was generated by the Symbolic Math Toolbox version 9.0.
%    06-Jun-2022 18:36:43

%in1=states=[uM;vM;0]
r = in1(3,:);
u = in1(1,:);
v = in1(2,:);
M = reshape([8.708580283640001e+5,0.0,0.0,0.0,1.180858028364e+6,-5.0e+5,0.0,1.0e+6,7.960656982913189e+7],[3,3]);
if nargout > 1
    t2 = abs(u);
    t3 = r+v;
    t4 = r.*2.0;
    t5 = r.*3.0;
    t6 = r.*4.0;
    t7 = r.*5.0;
    t8 = r.*6.0;
    t9 = r.*7.0;
    t11 = r.*8.0;
    t12 = r.*9.0;
    t13 = r.*1.0e+1;
    t14 = r.*1.1e+1;
    t15 = r.*1.2e+1;
    t16 = r.*1.3e+1;
    t17 = r.*1.4e+1;
    t18 = r.*1.5e+1;
    t19 = r.*1.6e+1;
    t20 = -v;
    t27 = r./2.0;
    t28 = r.*(3.0./2.0);
    t29 = r.*(5.0./2.0);
    t30 = r.*(7.0./2.0);
    t47 = r.*(9.0./2.0);
    t48 = r.*(1.1e+1./2.0);
    t49 = r.*(1.3e+1./2.0);
    t50 = r.*(1.5e+1./2.0);
    t51 = r.*(1.7e+1./2.0);
    t52 = r.*(1.9e+1./2.0);
    t53 = r.*(2.1e+1./2.0);
    t54 = r.*(2.3e+1./2.0);
    t55 = r.*(2.5e+1./2.0);
    t56 = r.*(2.7e+1./2.0);
    t57 = r.*(2.9e+1./2.0);
    t58 = r.*(3.1e+1./2.0);
    t59 = r.*(3.3e+1./2.0);
    t96 = r.*2.5e+5;
    t97 = u.*8.0e+4;
    t98 = v.*3.9e+5;
    t169 = r.*7.908580283640001e+5;
    C = reshape([0.0,t169,t3.*2.5e+5+v.*1.4e+5,-t169,0.0,-t97,t3.*-2.5e+5-v.*1.4e+5,t97,0.0],[3,3]);
end
if nargout > 2
    D = reshape([1.741716056728e+4,0.0,0.0,0.0,1.476072535455e+4,0.0,0.0,0.0,7.960656982913189e+6],[3,3]);
end
if nargout > 3
    t10 = abs(t3);
    t21 = r+t3;
    t22 = t3+t4;
    t23 = t3+t5;
    t24 = t3+t6;
    t25 = t3+t7;
    t26 = t3+t8;
    t37 = r+t20;
    t38 = t3+t9;
    t39 = t3+t11;
    t40 = t3+t12;
    t41 = t3+t13;
    t42 = t3+t14;
    t43 = t3+t15;
    t44 = t3+t16;
    t45 = t3+t17;
    t46 = t3+t18;
    t70 = t27+v;
    t71 = t28+v;
    t72 = t29+v;
    t73 = t30+v;
    t74 = t4+t20;
    t75 = t5+t20;
    t76 = t6+t20;
    t77 = t7+t20;
    t78 = t8+t20;
    t79 = t9+t20;
    t84 = t47+v;
    t85 = t48+v;
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
    t99 = t11+t20;
    t100 = t12+t20;
    t101 = t13+t20;
    t102 = t14+t20;
    t103 = t15+t20;
    t104 = t16+t20;
    t105 = t17+t20;
    t106 = t18+t20;
    t107 = t19+t20;
    t126 = t20+t27;
    t127 = t20+t28;
    t128 = t20+t29;
    t129 = t20+t30;
    t139 = t20+t47;
    t140 = t20+t48;
    t141 = t20+t49;
    t142 = t20+t50;
    t143 = t20+t51;
    t144 = t20+t52;
    t145 = t20+t53;
    t146 = t20+t54;
    t147 = t20+t55;
    t148 = t20+t56;
    t149 = t20+t57;
    t150 = t20+t58;
    t151 = t20+t59;
    t31 = abs(t21);
    t32 = abs(t22);
    t33 = abs(t23);
    t34 = abs(t24);
    t35 = abs(t25);
    t36 = abs(t26);
    t60 = abs(t37);
    t61 = abs(t38);
    t62 = abs(t39);
    t63 = abs(t40);
    t64 = abs(t41);
    t65 = abs(t42);
    t66 = abs(t43);
    t67 = abs(t44);
    t68 = abs(t45);
    t69 = abs(t46);
    t80 = abs(t70);
    t81 = abs(t71);
    t82 = abs(t72);
    t83 = abs(t73);
    t108 = abs(t84);
    t109 = abs(t85);
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
    t120 = abs(t74);
    t121 = abs(t75);
    t122 = abs(t76);
    t123 = abs(t77);
    t124 = abs(t78);
    t125 = abs(t79);
    t130 = abs(t99);
    t131 = abs(t100);
    t132 = abs(t101);
    t133 = abs(t102);
    t134 = abs(t103);
    t135 = abs(t104);
    t136 = abs(t105);
    t137 = abs(t106);
    t138 = abs(t107);
    t152 = abs(t126);
    t153 = abs(t127);
    t154 = abs(t128);
    t155 = abs(t129);
    t156 = abs(t139);
    t157 = abs(t140);
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
    t170 = t3.*t10.*8.45807816688347e+2;
    t171 = t37.*t60.*8.465623268214189e+2;
    et1 = t170-t171+t21.*t31.*8.395202322460815e+2+t22.*t32.*8.332326478038158e+2+t23.*t33.*8.271965667392404e+2+t24.*t34.*8.213281545931261e+2+t25.*t35.*8.154597424470116e+2;
    et2 = t26.*t36.*8.095913303008969e+2+t38.*t61.*8.037229181547824e+2+t39.*t62.*7.978545060086678e+2+t40.*t63.*7.919860938625533e+2+t41.*t64.*7.861176817164386e+2;
    et3 = t42.*t65.*7.80249269570324e+2+t43.*t66.*7.514940500543627e+2+t44.*t67.*6.54665249643472e+2+t45.*t68.*4.698985305341956e+2+t46.*t69.*1.339488636363636e+2+t70.*t80.*8.4895160890948e+2;
    et4 = t71.*t81.*8.426640244672141e+2+t72.*t82.*8.363764400249485e+2+t73.*t83.*8.30130772812298e+2+t84.*t108.*8.242623606661837e+2-t74.*t120.*8.59990057011688e+2;
    et5 = t85.*t109.*8.183939485200688e+2-t75.*t121.*8.920520561890112e+2+t86.*t110.*8.125255363739543e+2-t76.*t122.*9.596983741971574e+2+t87.*t111.*8.066571242278396e+2;
    et6 = t77.*t123.*(-1.197861800195613e+3)+t88.*t112.*8.007887120817251e+2-t78.*t124.*1.316802303365308e+3+t89.*t113.*7.949202999356104e+2-t79.*t125.*1.434187534665856e+3;
    et7 = t90.*t114.*7.890518877894958e+2+t91.*t115.*7.831834756433814e+2+t92.*t116.*7.735005956022923e+2+t93.*t117.*7.294875045064329e+2+t94.*t118.*5.711120769883375e+2;
    et8 = t95.*t119.*3.175972325143879e+2-t99.*t130.*1.555466180493116e+3-t100.*t131.*1.668055935415801e+3-t101.*t132.*1.024669058444421e+3-t102.*t133.*2.422999305369258e+2-t103.*t134.*1.695679012345679e+2-t104.*t135.*1.400833333333333e+2;
    et9 = t105.*t136.*(-1.173055555555556e+2)-t106.*t137.*9.452777777777778e+1-t107.*t138.*(2.87e+2./4.0)-t126.*t152.*8.552391933517455e+2-t127.*t153.*8.524607153688843e+2-t128.*t154.*8.725006982779683e+2-t129.*t155.*9.194225148788333e+2;
    et10 = t139.*t156.*(-1.125403801710639e+3)-t140.*t157.*1.26286188264802e+3-t141.*t158.*1.373548211752225e+3-t142.*t159.*1.494826857579486e+3-t143.*t160.*1.613081611022401e+3;
    et11 = t144.*t161.*(-1.425862089866242e+3)-t145.*t162.*4.25372953619864e+2-t146.*t163.*2.118916867355559e+2-t147.*t164.*1.514722222222222e+2-t148.*t165.*1.286944444444444e+2-t149.*t166.*1.059166666666667e+2-t150.*t167.*8.313888888888889e+1-t151.*t168.*6.036111111111111e+1+v.*abs(v).*8.520954011306128e+2;
    et12 = t170+t171+t21.*t31.*1.679040464492163e+3+t22.*t32.*2.499697943411447e+3+t23.*t33.*3.308786266956962e+3+t24.*t34.*4.10664077296563e+3+t25.*t35.*4.89275845468207e+3+t26.*t36.*5.667139312106278e+3;
    et13 = t38.*t61.*6.429783345238259e+3+t39.*t62.*7.180690554078011e+3+t40.*t63.*7.919860938625533e+3+t41.*t64.*8.647294498880825e+3+t42.*t65.*9.362991234843888e+3+t43.*t66.*9.769422650706714e+3;
    et14 = t44.*t67.*9.165313495008608e+3+t45.*t68.*7.048477958012933e+3+t46.*t69.*2.143181818181818e+3+t70.*t80.*4.2447580445474e+2+t71.*t81.*1.263996036700821e+3+t72.*t82.*2.090941100062371e+3;
    et15 = t73.*t83.*2.905457704843043e+3+t84.*t108.*3.709180622997827e+3+t74.*t120.*1.719980114023376e+3+t85.*t109.*4.501166716860378e+3+t75.*t121.*2.676156168567033e+3+t86.*t110.*5.281415986430703e+3;
    et16 = t76.*t122.*3.838793496788629e+3+t87.*t111.*6.049928431708797e+3+t77.*t123.*5.989309000978063e+3+t88.*t112.*6.806704052694663e+3+t78.*t124.*7.900813820191846e+3+t89.*t113.*7.551742849388299e+3;
    et17 = t79.*t125.*1.003931274266099e+4+t90.*t114.*8.285044821789706e+3+t91.*t115.*9.006609969898886e+3+t92.*t116.*9.668757445028654e+3+t93.*t117.*9.848081310836844e+3+t94.*t118.*8.281125116330895e+3;
    et18 = t95.*t119.*4.922757103973012e+3+t99.*t130.*1.244372944394493e+4+t100.*t131.*1.501250341874221e+4+t101.*t132.*1.024669058444421e+4+t102.*t133.*2.665299235906184e+3+t103.*t134.*2.034814814814815e+3+t104.*t135.*1.821083333333333e+3+t105.*t136.*1.642277777777778e+3;
    et19 = t106.*t137.*1.417916666666667e+3+t107.*t138.*1.148e+3+t126.*t152.*4.276195966758727e+2+t127.*t153.*1.278691073053326e+3+t128.*t154.*2.181251745694921e+3+t129.*t155.*3.217978802075917e+3+t139.*t156.*5.064317107697876e+3;
    et20 = t140.*t157.*6.945740354564111e+3+t141.*t158.*8.928063376389464e+3+t142.*t159.*1.121120143184614e+4+t143.*t160.*1.371119369369041e+4+t144.*t161.*1.35456898537293e+4;
    et21 = t145.*t162.*4.466416013008572e+3+t146.*t163.*2.436754397458893e+3+t147.*t164.*1.893402777777778e+3+t148.*t165.*1.737375e+3+t149.*t166.*1.535791666666667e+3+t150.*t167.*1.288652777777778e+3+t151.*t168.*9.959583333333333e+2;
    Dn = [t2.*u.*(3.228524982009454e+4./((log(t2.*3.35e+7)./log(1.0e+1)-2.0).^2+1.0./1.0e+3)+8.609399952025211e+3);et1+et2+et3+et4+et5+et6+et7+et8+et9+et10+et11;et12+et13+et14+et15+et16+et17+et18+et19+et20+et21];
end