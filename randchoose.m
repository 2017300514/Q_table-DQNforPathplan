function v=randchoose(N,m)
%  作用：从Ｎ这个数组中，随机选出ｍ个数字
%　 Example:
%   N =1 :100;
%   m =5;
%   randchoose(t,n)
%   ans =   6     2    14    99    74
%   randchoose(t,n)
%   ans =     63    89    10    97   100
    if length(m) >1
        error('m should be a number not a vector or matrix');
    end
    [row , col]= size(N);
    len = row * col;
    if m > len
        m = len;  % m > len 是返回整个数组的随机排序
    end
    a = rand(1,len);
    [b ,c]=sort(a);
    d = c(1:m);
    v = N(d);
end