function [ph,dph,ddph]=digit_new_Bezier_6th(Alpha,s,is_GPT) 


    S=[
        (1-s).^6
        6*s.*(1-s).^5
        15*s.^2*(1-s).^4
        20*s.^3*(1-s).^3
        15*s.^4*(1-s).^2
        6*s.^5*(1-s).^1
        s.^6
        ];

    dS=[
        6*(s - 1).^5
        - 30*s.*(s - 1).^4 - 6*(s - 1).^5
        30*s.*(s - 1).^4 + 60*s.^2*(s - 1).^3
        - 60*s.^2*(s - 1).^3 - 60*s.^3*(s - 1).^2
        15*s.^4*(2*s - 2) + 60*s.^3*(s - 1).^2
        - 30*s.^4*(s - 1) - 6*s.^5
        6*s.^5
        ];

    ddS=[
        30*(s - 1).^4
        - 120*s.*(s - 1).^3 - 60*(s - 1).^4
        240*s.*(s - 1).^3 + 30*(s - 1).^4 + 180*s.^2*(s - 1).^2
        - 120*s.*(s - 1).^3 - 60*s.^3*(2*s - 2) - 360*s.^2*(s - 1).^2
        120*s.^3*(2*s - 2) + 180*s.^2*(s - 1).^2 + 30*s.^4
        - 120*s.^3*(s - 1) - 60*s.^4
        30*s.^4
        ];
    %{
    a1=Alpha(1:7);                %torso z
    a2=Alpha(8:14);               %torso roll
    a3=Alpha(15:21);              %torso pitch
    a4=Alpha(22:28);              %torso yaw
    a5=Alpha(29:35);              %swing foot x
    a6=Alpha(36:42);              %swing foot y
    a7=Alpha(43:49);              %swing foot z
    a8=Alpha(50:56);              %swing roll
    a9=Alpha(57:63);              %swing pitch
    a10=Alpha(64:70);             %swing yaw
    %}
    if ~is_GPT
        a1=Alpha(1:7);                %swing foot x
        a2=Alpha(8:14);               %swing foot y
        a3=Alpha(15:21);              %swing foot z
        a4=Alpha(22:28);              %swing roll
        a5=Alpha(29:35);              %swing pitch
        a6=Alpha(36:42);              %swing yaw



        ph = [a1;a2;a3;a4;a5;a6]*S;
        dph = [a1;a2;a3;a4;a5;a6]*dS;
        ddph = [a1;a2;a3;a4;a5;a6]*ddS;
    else
        a1=Alpha(1:7);                %torso z
        a2=Alpha(8:14);               %torso roll
        a3=Alpha(15:21);              %torso pitch
        a4=Alpha(22:28);              %torso yaw
        a5=Alpha(29:35);              %swing foot x
        a6=Alpha(36:42);              %swing foot y
        a7=Alpha(43:49);              %swing foot z
        a8=Alpha(50:56);              %swing roll
        a9=Alpha(57:63);              %swing pitch
        a10=Alpha(64:70);             %swing yaw
        ph = [a1;a2;a3;a4;a5;a6;a7;a8;a9;a10]*S;
        dph = [a1;a2;a3;a4;a5;a6;a7;a8;a9;a10]*dS;
        ddph = [a1;a2;a3;a4;a5;a6;a7;a8;a9;a10]*ddS;
    end
    
    

end
%{


%}
