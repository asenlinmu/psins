function kfgencpp2(classname, Ft, Hk, up_first)
% 快速滤波计算法：根据矩阵Ft和Hk的非零元素直接展开，生成C++语言程序，参见我的文章:
%   "A rapid computation method for Kalman filtering in vehicular SINS/GPS integrated system"
% 滤波模型：dXt/dt = Ft*Xt+Qt, Zk = Hk*Xk+Rk
%   Ft--连续时间系统的一步转移矩阵, Hk--量测矩阵, Qt--系统噪声（要求对角阵）, Rk--量测噪声（要求对角阵）
% See also  nzFtHk.

    [M,N] = size(Hk);
    Pk = randn(N);  Pk = Pk*Pk';
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
    fprintf(fid, '\tvoid SetFt(CSINS& sins, CConfig& cfg);\n');
    fprintf(fid, '\tvoid SetHk(CSINS& sins);\n');
    % Qt
    fprintf(fid, '\tvoid SetQt(');
    for k=0:N-2, fprintf(fid, 'double q%d, ', k); end
    fprintf(fid, 'double q%d);\n', N-1);
    % Pk
    fprintf(fid, '\tvoid SetPk(');
    for k=0:N-2, fprintf(fid, 'double p%d, ', k); end
    fprintf(fid, 'double p%d);\n', N-1);
    % Rk, Zk
    fprintf(fid, '\tvoid SetRk(');
    for k=0:M-2, fprintf(fid, 'double r%d, ', k); end
    fprintf(fid, 'double r%d);\n', M-1);
    fprintf(fid, '\tvoid SetZk(');
    for k=0:M-2, fprintf(fid, 'double z%d, ', k); end
    fprintf(fid, 'double z%d);\n', M-1);
    % TimeUpdate
    fprintf(fid, '\tvoid TimeUpdate(double Ts);\n');
    % MeasUpdate
    fprintf(fid, '\tdouble MeasUpdate(double zweight=1.0, double fading=1.0);\n');
    fprintf(fid, '};\n\n#endif\n');
    fclose(fid);
%%  %%%%%%%%%%%%%%%%%%%%%%%%%%%% C++源程序 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    scpp = sprintf('%s.cpp', classname);
    fid = fopen(scpp,'wt');
    fprintf(fid, '#include "stdAfx.h"\n');
    fprintf(fid, '#include "%s.h"\n\n', classname);
    fprintf(fid, '#define  N        %d\n', N);
    fprintf(fid, '#define  M        %d\n', M);
    fprintf(fid, '#define  Ft(i,j)  Ft[(i)*N+(j)]\n');
    fprintf(fid, '#define  Fk(i,j)  Fk[(i)*N+(j)]\n');
    fprintf(fid, '#define  Hk(i,j)  Hk[(i)*N+(j)]\n');
    fprintf(fid, '#define  Pk(i,j)  Pk[(i)*N+(j)]\n');
    fprintf(fid, '#define  Bk(i,j)  Bk[(i)*N+(j)]\n');
    fprintf(fid, '\n');
    %% constructor
    fprintf(fid, 'C%s::C%s(void)\n{\n', classname, classname);
    fprintf(fid, '\tn = N, m = M;\n');
    fprintf(fid, '\tmemset(Ft, 0, N*N*sizeof(double));\n');
    fprintf(fid, '\tmemset(Hk, 0, M*N*sizeof(double));\n');
    fprintf(fid, '\tmemset(Xk, 0, N*sizeof(double));\n');
    fprintf(fid, '}\n\n');
    %% Ft
    fprintf(fid, 'void C%s::SetFt(CSINS& sins, CConfig& cfg)\n{\n\t//TODO: add your program requires here\n}\n\n', classname);
    %% Hk
    fprintf(fid, 'void C%s::SetHk(CSINS& sins)\n{\n\t//TODO: add your program requires here\n}\n\n', classname);
    %% Pk
    fprintf(fid, 'void C%s::SetPk(', classname);
    for k=0:N-2, fprintf(fid, 'double p%d, ', k); end
    fprintf(fid, 'double p%d)\n{\n', N-1);
    fprintf(fid, '\tmemset(Pk, 0, N*N*sizeof(double));\n\t');
    for k=0:N-1, fprintf(fid, 'Pk(%d,%d)=p%d*p%d; ', k, k, k, k); end
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
    %% Zk
    fprintf(fid, 'void C%s::SetZk(', classname);
    for k=0:M-2, fprintf(fid, 'double z%d, ', k); end
    fprintf(fid, 'double z%d)\n{\n\t', M-1);
    for k=0:M-1, fprintf(fid, 'Zk[%d]=z%d; ', k, k); end
    fprintf(fid, '\n}\n\n');
    %% %%%%%%%%%%%%%%% time update
    op_mul = 0; op_add = 0; 
    fprintf(fid, 'void C%s::TimeUpdate(double Ts)\n{\n', classname);
    fprintf(fid, '\tdouble Fk[N*N], Bk[N*N], X1[N];\n');
    % Fk = Ft*Ts
    Fk = Ft*0.1;
    fprintf(fid, '\t//Fk = Ft*Ts\n'); 
    for m=1:N
        for n=1:N
            if Ft(m,n)~=0, fprintf(fid, '\tFk(%2d,%2d)=Ft(%2d,%2d)*Ts;', m-1,n-1,m-1,n-1); op_mul = op_mul+1; end
        end
        if sum(abs(Ft(m,:)))>0,  fprintf(fid, '\n'); end
    end
    nz = op_mul;
    % Bk = Fk*Pk
    Bk = Fk*Pk;
    fprintf(fid, '\t//Bk = Fk*Pk\n'); 
    for m=1:N          
        for n=1:N
            if sum(abs(Fk(m,:)))==0, continue; end
            fprintf(fid, '\tBk(%2d,%2d)=', m-1,n-1); % Bk(m,n)=
            addfirst = 1;
            for k=1:N
                if Ft(m,k)~=0,
                    if addfirst==1,  addfirst = 0;
                        if k>n, fprintf(fid, 'Fk(%2d,%2d)*Pk(%2d,%2d)', m-1,k-1,n-1,k-1);  % 行>列，只用Pk的上三角
                        else    fprintf(fid, 'Fk(%2d,%2d)*Pk(%2d,%2d)', m-1,k-1,k-1,n-1); end
                    else
                        if k>n, fprintf(fid, '+Fk(%2d,%2d)*Pk(%2d,%2d)', m-1,k-1,n-1,k-1);
                        else    fprintf(fid, '+Fk(%2d,%2d)*Pk(%2d,%2d)', m-1,k-1,k-1,n-1); end
                    end
                    op_mul = op_mul+1;
                end
            end
%            if sum(abs(Fk(m,:)))==0, fprintf(fid, '0.0'); end
            fprintf(fid, ';\n');
        end
    end
    % Pk = (I+Fk)*Pk*(I+Fk)' = Pk + Bk + Bk' + Bk*Fk'
    fprintf(fid, '\t//Pk = (I+Fk)*Pk*(I+Fk)\'' = Pk + Bk + Bk\'' + Bk*Fk\''\n'); 
    Pk = randn(N,N); Bk = Fk*Pk;
    for m=1:N 
        if up_first==1,  k1 = 1; k2 = m;  else  k1=m; k2=N;  end
        for n=k1:k2       % 先计算下/上三角
            no0 = 0; % 统计非零元素
            if Bk(m,n)~=0||Bk(n,m)~=0, no0=1; end
            for k=1:N
                if Bk(m,k)~=0 && Fk(n,k)~=0,  no0=1; break; end
            end
            if no0==0, continue; end
            fprintf(fid, '\tPk(%2d,%2d)+=', m-1,n-1);
            if Bk(m,n)~=0, fprintf(fid, 'Bk(%2d,%2d)', m-1,n-1); op_add = op_add+1; end
            if Bk(n,m)~=0, fprintf(fid, '+Bk(%2d,%2d)', n-1,m-1); op_add = op_add+1; end
            for k=1:N
                if Bk(m,k)~=0 && Fk(n,k)~=0,  fprintf(fid, '+Bk(%2d,%2d)*Fk(%2d,%2d)', m-1,k-1,n-1,k-1);  op_add = op_add+1; op_mul = op_mul+1;  end
            end 
            fprintf(fid, ';\n');
        end
    end
%     fprintf(fid, '\tSymmetricPk(Pk);\n');
    % Pk = Pk + Qt*Ts
    fprintf(fid, '\t//Pk = Pk + Qt*Ts\n'); 
    fprintf(fid, '\t'); 
    for m=1:N, fprintf(fid, 'Pk(%d,%d)+=Qt[%d]*Ts; ', m-1,m-1, m-1); op_add = op_add+1; end
    fprintf(fid, '\n');
    % Xk = (I+Ft)*Xk
    fprintf(fid, '\t//Xk = (I+Fk)*Xk\n'); 
    for m=1:N
        fprintf(fid, '\tX1[%2d]=Xk[%2d]', m-1,m-1);
        for n=1:N
            if Ft(m,n)~=0    fprintf(fid, '+Fk(%2d,%2d)*Xk[%2d]', m-1,n-1,n-1); op_mul = op_mul+1;  end
        end
        fprintf(fid, ';\n');
    end
    fprintf(fid, '\tmemcpy(Xk, X1, N*sizeof(double));\n');
    fprintf(fid, '\t//Ft nonzero elements=%d, op_mul+=%d, op_add+=%d;\n}\n\n', nz, op_mul, op_add);
	%% %%%%%%%%%%%%%%% measure update
    % %%%%%sub measurement update function: SubMUpdt
    op_mul = 0; op_add = 0;
    fprintf(fid, 'static void SubMUpdt(double *Xk, double *Pk, double *Pxz, double Pzz, double r)\n{\n'); 
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
            fprintf(fid, 'Pk(%2d,%2d)-=Kk[%2d]*Pxz[%2d]; ', m-1,n-1, m-1, n-1); 
            op_add = op_add+1; op_mul = op_mul+1;
        end
        fprintf(fid, '\n');
    end
%     fprintf(fid, '\tSymmetricPk(Pk);\n');
    fprintf(fid, '}\n\n');
    op_add = op_add*M; op_mul = op_mul*M;
    % %%%%%%%%%%
    fprintf(fid, 'double C%s::MeasUpdate(double zweight, double fading)\n{\n', classname);
    fprintf(fid, '\tdouble Pxz[N], *Hi, Pzz, r;\n');
    fprintf(fid, '\tif(zweight<0.5)\n\t\treturn -1.0;\n');
    fprintf(fid, '\tzweight = zweight*zweight;\n');
    fprintf(fid, '\tHi=Hk, chi2=0.0;\n');
    for i=1:M
        Hi = Hk(i,:);  
        % Pxz = Pk*Hk'
        fprintf(fid, '\t//Pxz = Pk*Hk(%d,:)\''\n',i-1); 
        for m=1:N           
            fprintf(fid, '\tPxz[%2d]=', m-1); 
            for n=1:N
                if Hi(n)==1, op_add = op_add+1;
                    if m>n, fprintf(fid, '+Pk(%2d,%2d)', n-1,m-1);  % 只利用Pk上三角元素
                    else    fprintf(fid, '+Pk(%2d,%2d)', m-1,n-1); end
                elseif Hi(n)==-1, op_add = op_add+1;
                    if m>n, fprintf(fid, '-Pk(%2d,%2d)', n-1,m-1);
                    else    fprintf(fid, '-Pk(%2d,%2d)', m-1,n-1); end
                elseif Hi(n)~=0,  op_mul = op_mul+1; op_add = op_add+1;
                    if m>n, fprintf(fid, '+Pk(%2d,%2d)*Hi[%2d]', n-1,m-1,n-1);
                    else    fprintf(fid, '+Pk(%2d,%2d)*Hi[%2d]', m-1,n-1,n-1); end
                end
            end 
            fprintf(fid, ';\n');
        end
        % Pzz = Hk*Pxz + Ri
        fprintf(fid, '\t//Pzz = Hk(%d,:)*Pxz + Rk[%d]\n',i-1,i-1); 
        fprintf(fid, '\tPzz=');
        for m=1:N
            if Hi(m)==1,      fprintf(fid, '+Pxz[%2d]', m-1); op_add = op_add+1;
            elseif Hi(m)==-1, fprintf(fid, '-Pxz[%2d]', m-1); op_add = op_add+1;
            elseif Hi(m)~=0,  fprintf(fid, '+Hi[%2d]*Pxz[%2d]', m-1, m-1); op_mul = op_mul+1;
            end
        end
        fprintf(fid, '+Rk[%d]*zweight;\n', i-1); op_mul = op_mul+1;
        % Inno = Zk - Hk*Xk
        fprintf(fid, '\t//Innovation = Zk(%d) - Hk(%d,:)*Xk\n',i-1,i-1); 
        fprintf(fid, '\tr=Zk[%2d]',i-1);
        for m=1:N
            if Hi(m)==1,      fprintf(fid, '-Xk[%2d]', m-1); op_add = op_add+1;
            elseif Hi(m)==-1, fprintf(fid, '+Xk[%2d]', m-1); op_add = op_add+1;
            elseif Hi(m)~=0,  fprintf(fid, '-Hi[%2d]*Xk[%2d]', m-1, m-1); op_mul = op_mul+1;
            end
        end
        fprintf(fid, ';\n');
        fprintf(fid, '\tSubMUpdt(Xk, Pk, Pxz, Pzz, r);\n');
        fprintf(fid, '\tchi2+=r/Pzz*r;\n');
        if i<M
            fprintf(fid, '\tHi += N;\n');
        end
    end
    fprintf(fid, '\t//fading\n\tif(fading>1.0){ for(int i=0;i<N*N-1;i++){Pk[i]*=fading;} }\n');
    fprintf(fid, '\treturn chi2;\n\t//op_mul+=%d, op_add+=%d;\n}\n', op_mul, op_add);
    fclose(fid);
