function [q_rand, is_goal] = biasSample(self, varargin)
    %BIASSAMPLE Summary of this method goes here
    %   Detailed explanation goes here
    
    %   required:
    %   opts: 
    %   parameters:
    %       valid: decide to sample a valid one or not

    default_valid = false;
    default_th = 45;
    default_cut_p = 50;
    default_show_IRM = false;
    default_show_method = 2;
    
    p = inputParser;
    validPosNum = @(x) x>0 && isnumeric(x) && isscalar(x);
    validDegreeThreshold = @(a) a>0 && a<=360;
    validCutP = @(a) a>=0 && a<100;
    
    addParameter(p, 'valid', default_valid, @islogical);
    addParameter(p, 'th', default_th, validDegreeThreshold);
    addParameter(p, 'cut_p', default_cut_p, validCutP);
    addParameter(p, 'show_IRM', default_show_IRM, @islogical);
    addParameter(p, 'show_method', default_show_method, validPosNum);
    
    
    parse(p,varargin{:});
    args = p.Results;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % arguments
    valid = args.valid;
    th = args.th;
    cut_p = args.cut_p;
    show_IRM = args.show_IRM;
    show_method = args.show_method;

    
    q_rand = [];
    while isempty(q_rand)
        if rand() < self.bias
            t_next = self.max_t - rand() * self.t_step_size;
            t_next = max(0,t_next);
%             self.max_t = t_next;
            disp('biased');
        else
            t_next = min(1,self.max_t + rand() * self.t_step_size);
%             t_next = min(1,self.max_t + self.t_step_size);
        end
        if valid
            q_rand = self.config.sampleValidPlacement(t_next,'th',th,'cut_p',cut_p,'show_IRM',show_IRM,'show_method',show_method);
        else
            q_rand = self.config.samplePlacement(t_next,'th',th,'cut_p',cut_p,'show_IRM',show_IRM,'show_method',show_method);
        end
    end

    assert(q_rand.q(3)>=0 & q_rand.q(3)<=2*pi);
    is_goal = (q_rand.q(4)==1);
end