function kfgencpp1(classname, Ft, Hk, up_first)
% 快速滤波计算法：根据矩阵Ft和Hk的非零元素直接展开，生成C++语言程序，参见我的文章:
%   "A rapid computation method for Kalman filtering in vehicular SINS/GPS integrated system"
% 滤波模型：dXt/dt = Ft*Xt+Qt, Zk = Hk*Xk+Rk
%   Ft--连续时间系统的一步转移矩阵, Hk--量测矩阵, Qt--系统噪声（要求对角阵）, Rk--量测噪声（要求对角阵）
    [M,N] = size(Hk);
    if ~exist('up_first', 'var'), up_first = 0; end
%%  %%%%%%%%%%%%%%%%%%%%%%%%%%%% C++头文件 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    sh = sprintf('%s.h', classname);
    fid = fopen(sh, 'wt');
    fprintf(fid, '#ifndef _%s_h\n',classname);
    fprintf(fid, '#define _%s_h\n\n',classname);
    fprintf(fid, 'class C%s\n{\npublic:\n\tint n, m;\n', classname);
    fprintf(fid, '\tdouble Ft[%d*%d], Hk[%d*%d], Qt[%d], Rk[%d], Xk[%d], Pk[%d*%d], Zk[%d], chi2;\n\n', N,N, M,N, N, M, N, N,N, M);
    % constructor
    fprintf(fid, '\tC%s(void);\n', classname);
    % Ft, Hk
    fprintf(fid, '\tvoid SetFt();\n');
    fprintf(fid, '\tvoid SetHk();\n');
    % Qt
    fprintf(fid, '\tvoid SetQt(');
    for k=0:N-2, fprintf(fid, 'double q%d, ', k); end
    fprintf(fid, 'double q%d);\n', N-1);
    % Pk
    fprintf(fid, '\tvoid SetPk(');
    for k=0:N-2, fprintf(fid, 'double p%d, ', k); end
    fprintf(fid, 'double p%d);\n', N-1);
    % Rk
    fprintf(fid, '\tvoid SetRk(');
    for k=0:M-2, fprintf(fid, 'double r%d, ', k); end
    fprintf(fid, 'double r%d);\n', M-1);
    % TimeUpdate
    fprintf(fid, '\tvoid TimeUpdate(double Ts);\n');
    % MeasUpdate
    fprintf(fid, '\tvoid KXP(double *Pxz, double Pzz, double r);\n');
    fprintf(fid, '\tdouble MeasUpdate(');
    for k=0:M-1, fprintf(fid, 'double z%d, ', k); end
    fprintf(fid, 'double w=1.0, double s=1.0);\n');
    fprintf(fid, '};\n\n#endif\n');
    fclose(fid);
%%  %%%%%%%%%%%%%%%%%%%%%%%%%%%% C++源程序 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    scpp = sprintf('%s.cpp', classname);
    fid = fopen(scpp,'wt');
    fprintf(fid, '#include "stdAfx.h"\n');
    fprintf(fid, '#include "%s.h"\n\n', classname);
    fprintf(fid, '#define N %d\n', N);
    fprintf(fid, '#define M %d\n\n', M);
    %% constructor
    fprintf(fid, 'C%s::C%s(void)\n{\n', classname, classname);
    fprintf(fid, '\tn = N, m = M;\n');
    fprintf(fid, '\tmemset(Ft, 0, N*N*sizeof(double));\n');
    fprintf(fid, '\tmemset(Hk, 0, M*N*sizeof(double));\n');
    fprintf(fid, '\tmemset(Xk, 0, N*sizeof(double));\n');
    fprintf(fid, '}\n\n');
    %% Ft
    fprintf(fid, 'void C%s::SetFt()\n{\n\t//TODO: add your program requires here\n}\n\n', classname);
    %% Hk
    fprintf(fid, 'void C%s::SetHk()\n{\n\t//TODO: add your program requires here\n}\n\n', classname);
    %% Pk
    fprintf(fid, 'void C%s::SetPk(', classname);
    for k=0:N-2, fprintf(fid, 'double p%d, ', k); end
    fprintf(fid, 'double p%d)\n{\n', N-1);
    fprintf(fid, '\tmemset(Pk, 0, N*N*sizeof(double));\n\t');
    for k=0:N-1, fprintf(fid, 'Pk[%d*N+%d]=p%d*p%d; ', k, k, k, k); end
    fprintf(fid, '\n}\n\n');
    %% Qt
    fprintf(fid, 'void C%s::SetQt(', classname);
    for k=0:N-2, fprintf(fid, 'double q%d, ', k); end
    fprintf(fid, 'double q%d)\n{\n\t', N-1);
    for k=0:N-1, fprintf(fid, 'Qt[%d]=q%d*q%d; ', k, k, k); end
    fprintf(fid, '\n}\n\n');
    %% Rk
    fprintf(fid, 'void C%s::SetRk(', classname);
    for k=0:M-2, fprintf(fid, 'double r%d, ', k); end
    fprintf(fid, 'double r%d)\n{\n\t', M-1);
    for k=0:M-1, fprintf(fid, 'Rk[%d]=r%d*r%d; ', k, k, k); end
    fprintf(fid, '\n}\n\n');
    %% time update
    op_mul = 0; op_add = 0; 
    fprintf(fid, 'void C%s::TimeUpdate(double Ts)\n{\n', classname);
    fprintf(fid, '\tdouble Fk[N*N], B[N*N], X1[N];\n');
    % Fk = Ft*Ts
    fprintf(fid, '\t//Fk = Ft*Ts\n'); 
    Fk = Ft*0.1;
    for m=1:N
        for n=1:N
            if Ft(m,n)~=0, fprintf(fid, '\tFk[%2d*N+%2d]=Ft[%2d*N+%2d]*Ts;', m-1,n-1,m-1,n-1); op_mul = op_mul+1; end
        end
        if sum(abs(Ft(m,:)))>0,  fprintf(fid, '\n'); end
    end
    nz = op_mul;
    % B = Fk*Pk
    fprintf(fid, '\t//B = Fk*Pk\n'); 
    for m=1:N          
        for n=1:N
            fprintf(fid, '\tB[%2d*N+%2d]=', m-1,n-1); % B(m,n)=
            for k=1:N
                if Ft(m,k)~=0, fprintf(fid, '+Fk[%2d*N+%2d]*Pk[%2d*N+%2d]', m-1,k-1,k-1,n-1); op_mul = op_mul+1;  end
            end
            if sum(abs(Fk(m,:)))==0, fprintf(fid, '0.0'); end
            fprintf(fid, ';\n');
        end
    end
    % Pk = (I+Fk)*Pk*(I+Fk)' = Pk + B + B' + B*Fk'
    fprintf(fid, '\t//Pk = (I+Fk)*Pk*(I+Fk)\'' = Pk + B + B\'' + B*Fk\''\n'); 
    Pk = randn(N,N); B = Fk*Pk;
    for m=1:N 
        if up_first==1,  k1 = 1; k2 = m;  else  k1=m; k2=N;  end
        for n=k1:k2       % 先计算下/上三角
            fprintf(fid, '\tPk[%2d*N+%2d]+=B[%2d*N+%2d]+B[%2d*N+%2d]', m-1,n-1,m-1,n-1,n-1,m-1); op_add = op_add+2;
            for k=1:N
                if B(m,k)~=0 && Fk(n,k)~=0,  fprintf(fid, '+B[%2d*N+%2d]*Fk[%2d*N+%2d]', m-1,k-1,n-1,k-1);  op_add = op_add+1; op_mul = op_mul+1;  end
            end 
            fprintf(fid, ';\n');
        end
    end
    for m=1:N    % 再应用对称性计算P的上/下三角
        if up_first==1, k1=m+1; k2=N; else k1=1; k2=m-1; end
        for n=k1:k2  
            fprintf(fid, '\tPk[%2d*N+%2d]=Pk[%2d*N+%2d];', m-1,n-1, n-1,m-1);   
        end  
        if ~isempty(n), fprintf(fid, '\n'); end
    end
    % Pk = Pk + Q*Ts
    fprintf(fid, '\t//Pk = Pk + Qt*Ts\n'); 
    fprintf(fid, '\t'); 
    for m=1:N, fprintf(fid, 'Pk[%2d*N+%2d]+=Qt[%2d]*Ts; ', m-1,m-1, m-1); op_add = op_add+1; end
    fprintf(fid, '\n');
    % Xk = (I+Ft)*Xk
    fprintf(fid, '\t//Xk = (I+Fk)*Xk\n'); 
    for m=1:N          
        fprintf(fid, '\tX1[%2d]=Xk[%2d]', m-1,m-1);
        for n=1:N
            if Ft(m,n)~=0    fprintf(fid, '+Fk[%2d*N+%2d]*Xk[%2d]', m-1,n-1,n-1); op_mul = op_mul+1;  end
        end
        fprintf(fid, ';\n');
    end
    fprintf(fid, '\tmemcpy(Xk, X1, N*sizeof(double));\n');
    fprintf(fid, '\t//Ft nonzero elements=%d, op_mul+=%d, op_add+=%d;\n}\n\n', nz, op_mul, op_add);
	%% measure update
    % %%%%%sub-function KXP
    op_mul = 0; op_add = 0;
    fprintf(fid, 'void C%s::KXP(double *Pxz, double Pzz, double r)\n{\n', classname); 
    fprintf(fid, '\tdouble Kk[N];\n');
    % Kk = Pxz*Pzz^-1
    fprintf(fid, '\t//Kk = Pxz*Pzz^-1\n'); 
    for m=1:N, fprintf(fid, '\tKk[%2d]=Pxz[%2d]/Pzz;', m-1, m-1); op_mul = op_mul+1;  end
    fprintf(fid, '\n'); 
    % Xk = Xk + Kk*Inno;
    fprintf(fid, '\t//Xk = Xk + Kk*r\n'); 
    for m=1:N, fprintf(fid, '\tXk[%2d]+=Kk[%2d]*r;', m-1, m-1); op_add = op_add+1; op_mul = op_mul+1; end
    fprintf(fid, '\n'); 
    % Pk = Pk - Kk*Pxz';
    fprintf(fid, '\t//Pk = Pk - Kk*Pxz\''\n'); 
    for m=1:N    % 先计算上三角
        fprintf(fid, '\t');
        for n=m:N
            fprintf(fid, 'Pk[%2d*N+%2d]-=Kk[%2d]*Pxz[%2d]; ', m-1,n-1, m-1, n-1); 
            op_add = op_add+1; op_mul = op_mul+1;
        end
        fprintf(fid, '\n');
    end
    for m=1:N    % 再应用对称性计算P的下三角
        for n=1:m-1  
            fprintf(fid, '\tPk[%2d*N+%2d]=Pk[%2d*N+%2d];', m-1,n-1, n-1,m-1);   
        end  
        if ~isempty(n), fprintf(fid, '\n'); end
    end
    fprintf(fid, '}\n\n');
    op_add = op_add*M; op_mul = op_mul*M;
    % %%%%%%%%%%
    fprintf(fid, 'double C%s::MeasUpdate(', classname);
    for k=0:M-1, fprintf(fid, 'double z%d, ', k); end
    fprintf(fid, 'double w, double s)\n{\n');
    fprintf(fid, '\tdouble Pxz[N], *Hi, Pzz, r;\n\t');
    for k=0:M-1, fprintf(fid, 'Zk[%d]=z%d; ', k, k); end
    fprintf(fid, '\n\tHi=Hk, chi2=0.0;\n');
    for i=1:M
        Hi = Hk(i,:);  
        % Pxz = Pk*Hk'
        fprintf(fid, '\t//Pxz = Pk*Hk(%d,:)\''\n',i-1); 
        for m=1:N           
            fprintf(fid, '\tPxz[%2d]=', m-1); 
            for n=1:N
                if Hi(n)==1    fprintf(fid, '+Pk[%2d*N+%2d]      ', m-1,n-1); op_add = op_add+1;
                elseif Hi(n)==-1    fprintf(fid, '-Pk[%2d*N+%2d]      ', m-1,n-1); op_add = op_add+1;
                elseif Hi(n)~=0    fprintf(fid, '+Pk[%2d*N+%2d]*Hi[%2d]', m-1,n-1,n-1);  op_mul = op_mul+1;
                end
            end 
            fprintf(fid, ';\n');
        end
        % Pzz = Hk*Pxz + Ri
        fprintf(fid, '\t//Pzz = Hk(%d,:)*Pxz + Rk\n',i-1); 
        fprintf(fid, '\tPzz=');
        for m=1:N
            if Hi(m)==1, fprintf(fid, '      +Pxz[%2d]', m-1); op_add = op_add+1;
            elseif Hi(m)==-1, fprintf(fid, '      -Pxz[%2d]', m-1); op_add = op_add+1;
            elseif Hi(m)~=0, fprintf(fid, '+Hi[%2d]*Pxz[%2d]', m-1, m-1); op_mul = op_mul+1;
            end
        end
        fprintf(fid, '+Rk[%d]*w;\n', i-1); op_mul = op_mul+1;
        % Inno = Zk - Hk*Xk
        fprintf(fid, '\t//Innovation = Zk(%d) - Hk(%d,:)*X\n',i-1,i-1); 
        fprintf(fid, '\tr=Zk[%2d]',i-1);
        for m=1:N
            if Hi(m)==1, fprintf(fid, '      -Xk[%2d]', m-1); op_add = op_add+1;
            elseif Hi(m)==-1, fprintf(fid, '      +Xk[%2d]', m-1); op_add = op_add+1;
            elseif Hi(m)~=0 fprintf(fid, '-Hi[%2d]*Xk[%2d]', m-1, m-1); op_mul = op_mul+1;
            end
        end
        fprintf(fid, ';\n');
        fprintf(fid, '\tKXP(Pxz, Pzz, r);\n');
        fprintf(fid, '\tchi2+=r/Pzz*r;\n');
        if i<M
            fprintf(fid, '\tHi += N;\n');
        end
    end
    fprintf(fid, '\t//fading\n\tif(s>1.0){ for(int i=0;i<N*N-1;i++){Pk[i] *= s;} }\n');
    fprintf(fid, '\treturn chi2;\n\t//op_mul+=%d, op_add+=%d;\n}\n', op_mul, op_add);
    fclose(fid);
