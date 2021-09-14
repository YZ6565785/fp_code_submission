
foo(1, 0.5, 'show', true)


%% an exmaple for inputparser
function foo(a, varargin)
    % a is required num
    % b is a opt double
    
    default_b = 0.1;
    default_show = false;
%     expected_show = {true, false};

    p = inputParser;
    
    validPositiveNum = @(x) isnumeric(x) && isscalar(x) && (x > 0);
    
    addRequired(p,'a',validPositiveNum);
    addOptional(p,'b',default_b,validPositiveNum);
    addParameter(p,'show',default_show,@islogical)
    
    parse(p,a,varargin{:});
    args = p.Results;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    args
    
end