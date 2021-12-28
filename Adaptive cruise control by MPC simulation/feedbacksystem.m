function [Rs] = feedbacksystem(x_ego,v_ego,x_lead,v_lead,v_set,D_default,t_gap)

safe_distance = D_default + t_gap*v_ego;
real_distance = x_lead - x_ego;
dx = real_distance - safe_distance;        %inter distance error
v_rel = v_lead - v_ego;


V_des = v_lead;
if (dx<0)
    V_des = 0;
elseif (dx<=500)  %10*t_gap*abs(v_rel)
    if(v_lead >= v_set)
        V_des =  v_set;       %(v_ego-v_set)*(1 - exp(-dv/(0.4))) + v_set;
    elseif (v_lead < v_set)
        V_des = (v_set-v_lead)*(1 - exp(-dx/(100))) + v_lead; %2*t_gap*abs(v_rel)
    end
else
    V_des = v_set;
end

if dx < 500 %3*safe_distance
    X_des = x_lead - safe_distance;
else
    X_des = x_ego;
end

Rs = [X_des;V_des];

end