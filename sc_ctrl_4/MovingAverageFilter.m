classdef MovingAverageFilter < GeneralMath
    %   Class for moving average filter
    
    properties (SetAccess = private, GetAccess = private)
        fb % Filter buffer
    end
    
    properties (SetAccess = private, GetAccess = public)
        fo % Filter order
        sigs % Signal size
    end
    
    methods (Access = public)
        % Constructor
        function maf = MovingAverageFilter(filter_order, signal_size)
            maf.fo = filter_order;
            maf.sigs = signal_size;
            maf.fb(1:maf.sigs,:) = zeros(maf.sigs, 1);
            for i = 2:maf.fo
            	maf.fb((i-1)*maf.sigs+1:(i-1)*maf.sigs+maf.sigs, :) = zeros(maf.sigs, 1);
            end
        end
        function out = calcOutput(maf, ukm1)
            m = maf.sigs;
            
            maf.fb(1:m,:) = maf.fb(m+1:m+m,:);
            for i = 2:maf.fo-1
            	maf.fb((i-1)*m+1:(i-1)*m+m,:) = maf.fb(i*m+1:i*m+m,:);
            end
            
            maf.fb((maf.fo-1)*m+1:(maf.fo-1)*m+m,:) = ukm1;
            
            u1_sum = maf.fb(1);
            for i = 1:maf.fo-1
                u1_sum = u1_sum + maf.fb(i*m+1);
            end
            u2_sum = maf.fb(2);
            for i = 1:maf.fo-1
                u2_sum = u2_sum + maf.fb(i*m+2);
            end
            u3_sum = maf.fb(3);
            for i = 1:maf.fo-1
                u3_sum = u3_sum + maf.fb(i*m+3);
            end
            
            out = [u1_sum/maf.fo;
                   u2_sum/maf.fo;
                   u3_sum/maf.fo];
        end
    end
    
end

