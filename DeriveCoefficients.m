% State space model

A = [0,0,0,1,0,0;0,0,0,0,1,0;0,0,0,0,0,1;0,981/2480,0,-27/3100,0,0;0,0,-1635,0,0,0;0,-42183/6200,0,9/1550,0,0];
B = [0,0;0,0;0,0;37500/1271,0;0,10000/3;-25000/1271,0];
b1 = B(:,1);
b2 = B(:,2);
C = [1,0,0,0,0,0;0,1,0,0,0,0;0,0,1,0,0,0];
D = zeros(3,2);

% Calculate flat coordinates using the controllability matrix

P = [b1, A*b1, A*A*b1, A*A*A*b1, A*A*A*A*b1, A*A*A*A*A*b1];
P_inv = inv(P);
kappa = 1/P_inv(6,1);
lambda = ([0 0 0 0 0 kappa]*P_inv)'
T = [lambda';lambda'*A;lambda'*A*A; lambda'*A*A*A; lambda'*A*A*A*A; lambda'*A*A*A*A*A];
a = -lambda'*A*A*A*A*A*A*inv(T);

% Derive polynomial coefficients

w0 = 0;
wT = 0.2;
T_end = 1;
syms t
syms a7 a8 a9 a10 a11 a12 a13
z0 = w0;
zT = wT;
z = z0 + (zT - z0) * (a7*(t/T_end)^7 + a8*(t/T_end)^8 + a9*(t/T_end)^9 + a10*(t/T_end)^10 + a11*(t/T_end)^11 + a12*(t/T_end)^12 + a13*(t/T_end)^13)
dz = diff(z,t);
ddz = diff(dz,t);
dddz = diff(ddz,t);
ddddz = diff(dddz,t);
dddddz = diff(ddddz,t);
ddddddz = diff(dddddz,t);

t=0;
eqn1 = subs(z) == z0;
eqn2 = subs(dz) == 0;
eqn3 = subs(ddz) == 0;
eqn4 = subs(dddz) == 0;
eqn5 = subs(ddddz) == 0;
eqn6 = subs(dddddz) == 0;
eqn7 = subs(ddddddz) == 0;
t=T_end;
eqn8 = subs(z) == zT;
eqn9 = subs(dz) == 0;
eqn10 = subs(ddz) == 0;
eqn11 = subs(dddz) == 0;
eqn12 = subs(ddddz) == 0;
eqn13 = subs(dddddz) == 0;
eqn14 = subs(ddddddz) == 0;

[A,B] = equationsToMatrix([eqn1,eqn2,eqn3,eqn4,eqn5,eqn6,eqn7,eqn8,eqn9,eqn10,eqn11,eqn12,eqn13,eqn14], [a7 a8 a9 a10 a11 a12 a13])
X = linsolve(A,B)

% Plot trajectories

syms t T_end
T_end = 1
zquer = w0 + (wT - w0) * (1716*(t/T_end)^7 +  -9009*(t/T_end)^8 +    20020*(t/T_end)^9 + -24024*(t/T_end)^10 +   16380*(t/T_end)^11 +   -6006*(t/T_end)^12 +  924*(t/T_end)^13);
dzquer = diff(zquer,t);
ddzquer = diff(dzquer,t);
dddzquer = diff(ddzquer,t);
ddddzquer = diff(dddzquer,t);
dddddzquer = diff(ddddzquer,t);
ddddddzquer = diff(dddddzquer,t);
zquervec = [zquer;dzquer;ddzquer;dddzquer;ddddzquer;dddddzquer];
xquer = inv(T)* zquervec
uquer = 1/kappa * (ddddddzquer + a*zquervec);
tt=linspace(0,T_end,100);
for idx= 1:100
    tidx=tt(idx);
    sub = subs(zquer,t,tidx);
    zquerv(idx) = double(sub);
    sub = subs(xquer(1),t,tidx);
    xquer1v(idx) = double(sub);
    sub = subs(xquer(2),t,tidx);
    xquer2v(idx) = double(sub);
    sub = subs(xquer(3),t,tidx);
    xquer3v(idx) = double(sub);
    sub = subs(xquer(4),t,tidx);
    xquer4v(idx) = double(sub);
    sub = subs(xquer(5),t,tidx);
    xquer5v(idx) = double(sub);
    sub = subs(xquer(6),t,tidx);
    xquer6v(idx) = double(sub);
    sub = subs(uquer,t,tidx);
    uquerv(idx) = double(sub);
    tv(idx)=tidx;
end
figure(1)
subplot(3,2,1)
plot(tv,zquerv)
subplot(3,1,1)
plot(tv,xquer1v)
title('Robot position')
subplot(3,1,2)
plot(tv,xquer2v)
title('Beam angle')
subplot(3,1,3)
plot(tv,xquer3v)
title('Ball position')
print -deps epsFig
%subplot(3,2,7)
figure(2)
plot(tv,uquerv)
print -deps epsFig

% Save coefficients

w0 = 0
wT = 0.2
order = [12,11,10,9,8,7,6,5,4,3,2,1];
u1 = [(1011384551389116279375*w0)/36893488147419103232 - (1011384551389116279375*wT)/36893488147419103232, (5266198448319017337403275*wT)/9149585060559937601536 - (5266198448319017337403275*w0)/9149585060559937601536, (820778839914287620130753273361648400306441489138209289678674967586255922975*w0)/213954136750507477410477431730587789025984378137281983210038250937778176 - (820778839914287620130753273361648400306441489138209289678674967586255922975*wT)/213954136750507477410477431730587789025984378137281983210038250937778176, (4923242672798446695031786653973917025830691322699064169429704492380204692875*wT)/427908273501014954820954863461175578051968756274563966420076501875556352 - (4923242672798446695031786653973917025830691322699064169429704492380204692875*w0)/427908273501014954820954863461175578051968756274563966420076501875556352, (1573555013189173307112566308604702498740140860746777372629752998526175278475*w0)/106977068375253738705238715865293894512992189068640991605019125468889088 - (1573555013189173307112566308604702498740140860746777372629752998526175278475*wT)/106977068375253738705238715865293894512992189068640991605019125468889088, (275211172919769875758134094174482710014113837479084716289240027971834219125*w0)/53488534187626869352619357932646947256496094534320495802509562734444544 - (275211172919769875758134094174482710014113837479084716289240027971834219125*wT)/53488534187626869352619357932646947256496094534320495802509562734444544, (5049430764256931909249346555042778574083916799944755626794538651487104163725*wT)/106977068375253738705238715865293894512992189068640991605019125468889088 - (5049430764256931909249346555042778574083916799944755626794538651487104163725*w0)/106977068375253738705238715865293894512992189068640991605019125468889088, (32601694659793206935794509417262441699609220633778308386311690806811381833475*w0)/427908273501014954820954863461175578051968756274563966420076501875556352 - (32601694659793206935794509417262441699609220633778308386311690806811381833475*wT)/427908273501014954820954863461175578051968756274563966420076501875556352, (437988613275138939642066001493492112251988709108601137169449031965713403125*wT)/6901746346790563787434755862277025452451108972170386555162524223799296 - (437988613275138939642066001493492112251988709108601137169449031965713403125*w0)/6901746346790563787434755862277025452451108972170386555162524223799296, (289736985141243206602793024134473230625*w0)/10384593717069655257060992658440192 - (289736985141243206602793024134473230625*wT)/10384593717069655257060992658440192, (26279469823656077341241060372321206125*wT)/5192296858534827628530496329220096 - (26279469823656077341241060372321206125*w0)/5192296858534827628530496329220096, (36823171168130880507853335496664625*wT)/10384593717069655257060992658440192 - (36823171168130880507853335496664625*w0)/10384593717069655257060992658440192];
u = double(u1(order));
order = [13,12,11,10,9,8,7,6,5,4,3,2,1,24,23,22,21,20,19,18,17,16,15,14,35,34,33,32,31,30,29,28,27,26,25,46,45,44,43,42,41,40,39,38,37,36,57,56,55,54,53,52,51,50,49,48,47,68,67,66,65,64,63,62,61,60,59,58];
x1 = [w0, (91300322845796259195*wT)/89202980794122492566142873090593446023921664 - (91300322845796259195*w0)/89202980794122492566142873090593446023921664, (6012435571074261344189546211297078160405550685*w0)/44601490397061246283071436545296723011960832 - (6012435571074261344189546211297078160405550685*wT)/44601490397061246283071436545296723011960832, (63130573496279744114011199338151375026148421585*wT)/44601490397061246283071436545296723011960832 - (63130573496279744114011199338151375026148421585*w0)/44601490397061246283071436545296723011960832, (63130573496279744114034266473643672430481484823*w0)/11150372599265311570767859136324180752990208 - (63130573496279744114034266473643672430481484823*wT)/11150372599265311570767859136324180752990208, (505044587970237952912509042501621039230172039987*wT)/44601490397061246283071436545296723011960832 - (505044587970237952912509042501621039230172039987*w0)/44601490397061246283071436545296723011960832, (232291521937663211178157640994024139086853794843*w0)/22300745198530623141535718272648361505980416 - (232291521937663211178157640994024139086853794843*wT)/22300745198530623141535718272648361505980416, (208398532437897662451728118822880346046488449701*w0)/89202980794122492566142873090593446023921664 - (208398532437897662451728118822880346046488449701*wT)/89202980794122492566142873090593446023921664, (3229629087763083108595873030999252525116880385*wT)/174224571863520493293247799005065324265472 - (3229629087763083108595873030999252525116880385*w0)/174224571863520493293247799005065324265472, (16742284457796865323506496525352571900229606879*w0)/696898287454081973172991196020261297061888 - (16742284457796865323506496525352571900229606879*wT)/696898287454081973172991196020261297061888, (5707596974248931360286797707987193447798955891*wT)/348449143727040986586495598010130648530944 - (5707596974248931360286797707987193447798955891*w0)/348449143727040986586495598010130648530944, (4185571114449216330876985144484000594897937921*w0)/696898287454081973172991196020261297061888 - (4185571114449216330876985144484000594897937921*wT)/696898287454081973172991196020261297061888, 924*wT - 924*w0, (1213230839868007845885*wT)/85070591730234615865843651857942052864 - (1213230839868007845885*w0)/85070591730234615865843651857942052864, (2173857087734614169052165*w0)/10889035741470030830827987437816582766592 - (2173857087734614169052165*wT)/10889035741470030830827987437816582766592, (1391090796043463191467015225*wT)/1393796574908163946345982392040522594123776 - (1391090796043463191467015225*w0)/1393796574908163946345982392040522594123776, (558076148593228844298856448945389217284933370261*w0)/75961913332494935075856040366208481379745792 - (558076148593228844298856448945389217284933370261*wT)/75961913332494935075856040366208481379745792, (1953266520076300954684286494846221069932806011811*wT)/37980956666247467537928020183104240689872896 - (1953266520076300954684286494846221069932806011811*w0)/37980956666247467537928020183104240689872896, (1395190371483072110552210434046239724008400732639*w0)/9495239166561866884482005045776060172468224 - (1395190371483072110552210434046239724008400732639*wT)/9495239166561866884482005045776060172468224, (33484568915593730656768284512404744191286455491745*wT)/151923826664989870151712080732416962759491584 - (33484568915593730656768284512404744191286455491745*w0)/151923826664989870151712080732416962759491584, (13951903714830721109377316427310732647270418970615*w0)/75961913332494935075856040366208481379745792 - (13951903714830721109377316427310732647270418970615*wT)/75961913332494935075856040366208481379745792, (6138837634525517290126783847061886924972131088651*wT)/75961913332494935075856040366208481379745792 - (6138837634525517290126783847061886924972131088651*w0)/75961913332494935075856040366208481379745792, (4157991324314689972140186623636037815947*w0)/282980178790148105754912049942495232 - (4157991324314689972140186623636037815947*wT)/282980178790148105754912049942495232, (22057997440968149661*wT)/5192296858534827628530496329220096 - (22057997440968149661*w0)/5192296858534827628530496329220096, (486935055177579913905*w0)/713623846352979940529142984724747568191373312 - (486935055177579913905*wT)/713623846352979940529142984724747568191373312, (32066323045729384541842734858592003050320212695*wT)/356811923176489970264571492362373784095686656 - (32066323045729384541842734858592003050320212695*w0)/356811923176489970264571492362373784095686656, (336696391980158537720046945476258807489794113075*w0)/356811923176489970264571492362373784095686656 - (336696391980158537720046945476258807489794113075*wT)/356811923176489970264571492362373784095686656, (336696391980158537753814356504723001629611145349*wT)/89202980794122492566142873090593446023921664 - (336696391980158537753814356504723001629611145349*w0)/89202980794122492566142873090593446023921664, (2693571135841268302374322094541103215905320671129*w0)/356811923176489970264571492362373784095686656 - (2693571135841268302374322094541103215905320671129*wT)/356811923176489970264571492362373784095686656, (1442984537057822305119654671730264039334461771729*wT)/178405961588244985132285746181186892047843328 - (1442984537057822305119654671730264039334461771729*w0)/178405961588244985132285746181186892047843328, (3174565981527209072405115431836874240859827478225*w0)/713623846352979940529142984724747568191373312 - (3174565981527209072405115431836874240859827478225*wT)/713623846352979940529142984724747568191373312, (344462454592796123667544850115237844136268465*wT)/348449143727040986586495598010130648530944 - (344462454592796123667544850115237844136268465*w0)/348449143727040986586495598010130648530944, (527513350743542600759493171*w0)/1393796574908163946345982392040522594123776 - (527513350743542600759493171*wT)/1393796574908163946345982392040522594123776, (10273379718357551829033*w0)/696898287454081973172991196020261297061888 - (10273379718357551829033*wT)/696898287454081973172991196020261297061888, (12046928654849835747*wT)/1393796574908163946345982392040522594123776 - (12046928654849835747*w0)/1393796574908163946345982392040522594123776, (466253450495326899405*w0)/1152921504606846976 - (466253450495326899405*wT)/1152921504606846976, (963296104374385743927172792703858582337495*wT)/170141183460469231731687303715884105728 - (963296104374385743927172792703858582337495*w0)/170141183460469231731687303715884105728, (4932076054396855008908926918463819842908261585*w0)/174224571863520493293247799005065324265472 - (4932076054396855008908926918463819842908261585*wT)/174224571863520493293247799005065324265472, (378783440977678464684381775744264071968116268919*wT)/5575186299632655785383929568162090376495104 - (378783440977678464684381775744264071968116268919*w0)/5575186299632655785383929568162090376495104, (406510163390910619561775873872589549597433290241*w0)/5575186299632655785383929568162090376495104 - (406510163390910619561775873872589549597433290241*wT)/5575186299632655785383929568162090376495104, (26049816554737207806466009939730350684526801517*w0)/1393796574908163946345982392040522594123776 - (26049816554737207806466009939730350684526801517*wT)/1393796574908163946345982392040522594123776, (465066588637883967637805700594661804126826133495*wT)/2787593149816327892691964784081045188247552 - (465066588637883967637805700594661804126826133495*w0)/2787593149816327892691964784081045188247552, (1339382756623749225880519695579488162379021573345*w0)/5575186299632655785383929568162090376495104 - (1339382756623749225880519695579488162379021573345*wT)/5575186299632655785383929568162090376495104, (1004537067467811919410476384968310308893045612771*wT)/5575186299632655785383929568162090376495104 - (1004537067467811919410476384968310308893045612771*w0)/5575186299632655785383929568162090376495104, (200907413493562383882095285877283325107872516297*w0)/2787593149816327892691964784081045188247552 - (200907413493562383882095285877283325107872516297*wT)/2787593149816327892691964784081045188247552, 12012*wT - 12012*w0, (594824777491629980295*w0)/10889035741470030830827987437816582766592 - (594824777491629980295*wT)/10889035741470030830827987437816582766592, (1066223276243682438146265*wT)/1393796574908163946345982392040522594123776 - (1066223276243682438146265*w0)/1393796574908163946345982392040522594123776, (11161522971864576882339789535264309986735413329225*w0)/303847653329979740303424161464833925518983168 - (11161522971864576882339789535264309986735413329225*wT)/303847653329979740303424161464833925518983168, (23439198240915611455048880230439109109228020926793*wT)/75961913332494935075856040366208481379745792 - (23439198240915611455048880230439109109228020926793*w0)/75961913332494935075856040366208481379745792, (39065330401526019095665220837481521767318123821065*w0)/37980956666247467537928020183104240689872896 - (39065330401526019095665220837481521767318123821065*wT)/37980956666247467537928020183104240689872896, (133938275662374922630803706479295576360494228498369*wT)/75961913332494935075856040366208481379745792 - (133938275662374922630803706479295576360494228498369*w0)/75961913332494935075856040366208481379745792, (502268533733905959961669839644385300782356603795215*w0)/303847653329979740303424161464833925518983168 - (502268533733905959961669839644385300782356603795215*wT)/303847653329979740303424161464833925518983168, (122776752690510345812566974260740247729684166971735*wT)/151923826664989870151712080732416962759491584 - (122776752690510345812566974260740247729684166971735*w0)/151923826664989870151712080732416962759491584, (93671228554161335709198066998177643639384709*w0)/579543406162223320586059878282230235136 - (93671228554161335709198066998177643639384709*wT)/579543406162223320586059878282230235136, (149523882092814499955685*wT)/2658455991569831745807614120560689152 - (149523882092814499955685*w0)/2658455991569831745807614120560689152, (19616364991367881737*wT)/5316911983139663491615228241121378304 - (19616364991367881737*w0)/5316911983139663491615228241121378304, (77708908415887794045*wT)/288230376151711744 - (77708908415887794045*w0)/288230376151711744, (1254291802570814407319005312150463023485*w0)/332306998946228968225951765070086144 - (1254291802570814407319005312150463023485*wT)/332306998946228968225951765070086144, (102751584466601116257877913293178361693348885*wT)/5444517870735015415413993718908291383296 - (102751584466601116257877913293178361693348885*w0)/5444517870735015415413993718908291383296, (31565286748139862918449086534407578573603060163*w0)/696898287454081973172991196020261297061888 - (31565286748139862918449086534407578573603060163*wT)/696898287454081973172991196020261297061888, (5050445879702378067918791373792682749986603579691*wT)/89202980794122492566142873090593446023921664 - (5050445879702378067918791373792682749986603579691*w0)/89202980794122492566142873090593446023921664, (1587282990763604536202557820684632043327262096129*w0)/44601490397061246283071436545296723011960832 - (1587282990763604536202557820684632043327262096129*wT)/44601490397061246283071436545296723011960832, (793641495381802268930023672972900196379479476485*wT)/89202980794122492566142873090593446023921664 - (793641495381802268930023672972900196379479476485*w0)/89202980794122492566142873090593446023921664, (84402206570149329595114916535*w0)/22300745198530623141535718272648361505980416 - (84402206570149329595114916535*wT)/22300745198530623141535718272648361505980416, (14341022188775199029883771*w0)/89202980794122492566142873090593446023921664 - (14341022188775199029883771*wT)/89202980794122492566142873090593446023921664, (6611526926859603381633*w0)/44601490397061246283071436545296723011960832 - (6611526926859603381633*wT)/44601490397061246283071436545296723011960832, (15746213260609909077*w0)/89202980794122492566142873090593446023921664 - (15746213260609909077*wT)/89202980794122492566142873090593446023921664];        
x = double(x1(order));
size(x,2)
size(u)
t  = 0.5;
T_start = 0;
xquer = [x(1)*(t-T_start)^13+x(2)*(t-T_start)^12+x(3)*(t-T_start)^11+x(4)*(t-T_start)^10+x(5)*(t-T_start)^9+x(6)*(t-T_start)^8+x(7)*(t-T_start)^7+x(8)*(t-T_start)^6+x(9)*(t-T_start)^5+x(10)*(t-T_start)^4+x(11)*(t-T_start)^3+x(12)*(t-T_start)^2+x(13);...
x(14)*(t-T_start)^12+x(15)*(t-T_start)^11+x(16)*(t-T_start)^10+x(17)*(t-T_start)^9+x(18)*(t-T_start)^8+x(19)*(t-T_start)^7+x(20)*(t-T_start)^6+x(21)*(t-T_start)^5+x(22)*(t-T_start)^4+x(23)*(t-T_start)^3+x(24)*(t-T_start)^2;...
x(25)*(t-T_start)^12+x(26)*(t-T_start)^11+x(27)*(t-T_start)^10+x(28)*(t-T_start)^9+x(29)*(t-T_start)^8+x(30)*(t-T_start)^7+x(31)*(t-T_start)^6+x(32)*(t-T_start)^5+x(33)*(t-T_start)^4+x(34)*(t-T_start)^3+x(35)*(t-T_start)^2;...
x(36)*(t-T_start)^12+x(37)*(t-T_start)^11+x(38)*(t-T_start)^10+x(39)*(t-T_start)^9+x(40)*(t-T_start)^8+x(41)*(t-T_start)^7+x(42)*(t-T_start)^6+x(43)*(t-T_start)^5+x(44)*(t-T_start)^4+x(45)*(t-T_start)^3+x(46)*(t-T_start)^2;...
x(47)*(t-T_start)^12+x(48)*(t-T_start)^11+x(49)*(t-T_start)^10+x(50)*(t-T_start)^9+x(51)*(t-T_start)^8+x(52)*(t-T_start)^7+x(53)*(t-T_start)^6+x(54)*(t-T_start)^5+x(55)*(t-T_start)^4+x(56)*(t-T_start)^3+x(57)*(t-T_start)^2;...
x(58)*(t-T_start)^12+x(59)*(t-T_start)^11+x(60)*(t-T_start)^10+x(61)*(t-T_start)^9+x(62)*(t-T_start)^8+x(63)*(t-T_start)^7+x(64)*(t-T_start)^6+x(65)*(t-T_start)^5+x(66)*(t-T_start)^4+x(67)*(t-T_start)^3+x(68)*(t-T_start)^2];
uquer = u(1)*(t-T_start)^12+u(2)*(t-T_start)^11+u(3)*(t-T_start)^10+u(4)*(t-T_start)^9+u(5)*(t-T_start)^8+u(6)*(t-T_start)^7+u(7)*(t-T_start)^6+u(8)*(t-T_start)^5+u(9)*(t-T_start)^4+u(10)*(t-T_start)^3+u(11)*(t-T_start)^2+u(12)*(t-T_start);
  
