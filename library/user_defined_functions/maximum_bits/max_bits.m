function [out] = max_bits(m)
%MAX_BITS Summary of this function goes here
%   Detailed explanation goes here
% out = ((rem(m/(rem(m,255)+1)/255, 255)*8 + 7-86/(rem(m,255)+12)));
out = ceil(m*(log(10)/log(2)));
end

