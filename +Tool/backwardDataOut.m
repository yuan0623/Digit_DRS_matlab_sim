function [t_global_filtered, V_global_filtered, ...
    contact_indictor_global_filtered,...
    AM_prediction_global_filtered, x0_LIP_sagittal_global_filtered, ...
    x0_LIP_lateral_global_filtered, AM_COM_global_filtered]= backwardDataOut()
    global t_global V_global contact_indictor_global AM_prediction_global x0_LIP_lateral_global ...
        x0_LIP_sagittal_global AM_COM_global
    t_global_filtered = t_global;
    V_global_filtered = V_global;
    contact_indictor_global_filtered = contact_indictor_global;
    AM_prediction_global_filtered = AM_prediction_global;
    x0_LIP_sagittal_global_filtered = x0_LIP_sagittal_global;
    x0_LIP_lateral_global_filtered = x0_LIP_lateral_global;
    AM_COM_global_filtered = AM_COM_global;

    i = 2;
    while i<length(t_global_filtered)
        if t_global_filtered(i)<=t_global_filtered(i-1)
            t_global_filtered(i) = [];
            V_global_filtered(i) = [];
            contact_indictor_global_filtered(i) = [];
            AM_prediction_global_filtered(:,i) = [];
            x0_LIP_sagittal_global_filtered(:,i) = [];
            x0_LIP_lateral_global_filtered(:,i) = [];
            %AM_COM_global_filtered(:,i) = [];
        else 
           i = i+1
        end

    end
    


end