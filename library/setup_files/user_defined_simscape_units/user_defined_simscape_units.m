classdef user_defined_simscape_units
    methods(Static)
        function defineUnits()
            pm_addunit('percent', 0.01, '1');
        end
    end
end
