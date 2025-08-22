function params = getBaseParams(modelParameters, iAgent)
    if nargin < 1 || isempty(modelParameters)
        params = defaultBlueROV2Params(); return;
    end
    s = modelParameters;
    if isstruct(s)
        if numel(s) >= iAgent, si = s(iAgent); else, si = s(1); end
        if isfield(si,'params'), params = si.params; return; end
        needed = {'m','xg','yg','zg','Ixx','Iyy','Izz','Ixy','Ixz','Iyz', ...
                  'X_u_dot','Y_v_dot','Z_w_dot','K_p_dot','M_q_dot','N_r_dot', ...
                  'X_u','Y_v','Z_w','K_p','M_q','N_r', ...
                  'X_uu','Y_vv','Z_ww','K_pp','M_qq','N_rr'};
        if all(isfield(si,needed)), params = rmfield(si,setdiff(fieldnames(si),needed)); return; end
    end
    params = defaultBlueROV2Params();
end