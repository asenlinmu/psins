function KF_rapid_cpp(fname, A, C, Q)
% 快速滤波计算法：根据矩阵A、C和Q的非零元素直接展开，生成C语言程序，参见:
%   "A rapid computation method for Kalman filtering in vehicular SINS/GPS integrated system"
% 滤波模型：dXt = At*Xt+Qt, Zk = Ck*Xk+Rk
%   A--连续时间系统的一步转移矩阵, C--量测矩阵, Q--系统噪声（要求对角阵）, R--量测噪声（要求对角阵）
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% C++头文件 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    sh = sprintf('%s.h', fname);
    fid = fopen(sh,'wt');
    fprintf(fid, '#ifndef _%s_h\n',fname);
    fprintf(fid, '#define _%s_h\n\n',fname);
    fprintf(fid, 'void SysUpdate(double *X, double *P, double *A, double *Q, double Ts);\n');
    fprintf(fid, 'double MeasUpdate(double *X, double *P, double *C, double *Z, double *R);\n\n');
    fprintf(fid, '#endif\n');
    fclose(fid);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% C++源程序 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    [M,N] = size(C);
    scpp = sprintf('%s.cpp', fname);
    fid = fopen(scpp,'wt');
    fprintf(fid, '#include "stdAfx.h"\n');
    fprintf(fid, '#include "navi.h"\n');
    fprintf(fid, '#include "%s"\n\n',sh);
    fprintf(fid, '#define M %d\n', M);
    fprintf(fid, '#define N %d\n\n', N);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    op_mul = 0; op_add = 0; 
    fprintf(fid, 'void SysUpdate(double *X, double *P, double *A, double *Q, double Ts)\n{\n');
    fprintf(fid, '\tdouble F[N*N], B[N*N], X1[N];\n');
    % F = A*Ts
    fprintf(fid, '\t//F = A*Ts\n'); 
    F = A*0.1;
    for m=1:N
        for n=1:N
            if A(m,n)~=0 fprintf(fid, '\tF[%2d*N+%2d]=A[%2d*N+%2d]*Ts;', m-1,n-1,m-1,n-1); op_mul = op_mul+1; end
        end
        if sum(abs(A(m,:)))>0  fprintf(fid, '\n'); end
    end
    fprintf(fid, '\t//No-zero elements=%d;\n', op_mul);
    % B = F*P
    fprintf(fid, '\t//B = F*P\n'); 
    for m=1:N          
        for n=1:N
            fprintf(fid, '\tB[%2d*N+%2d]=', m-1,n-1); % B(m,n)=
            for k=1:N
                if F(m,k)~=0 fprintf(fid, '+F[%2d*N+%2d]*P[%2d*N+%2d]', m-1,k-1,k-1,n-1); op_mul = op_mul+1;  end
            end
            if sum(abs(F(m,:)))==0 fprintf(fid, '0.0'); end
            fprintf(fid, ';\n');
        end
    end
    % P1 = P + B + B^T + B*F^T
    fprintf(fid, '\t//P = P + B + B^T + B*F^T\n'); 
    up_first = 0;
    P = randn(N,N); B = F*P;
    for m=1:N 
        if up_first==1  k1 = 1; k2 = m;  else  k1=m; k2=N;  end
        for n=k1:k2       % 先计算下/上三角
            fprintf(fid, '\tP[%2d*N+%2d]+=B[%2d*N+%2d]+B[%2d*N+%2d]', m-1,n-1,m-1,n-1,n-1,m-1); op_add = op_add+2;
            for k=1:N
                if B(m,k)~=0 & F(n,k)~=0  fprintf(fid, '+B[%2d*N+%2d]*F[%2d*N+%2d]', m-1,k-1,n-1,k-1);  op_add = op_add+1; op_mul = op_mul+1;  end
            end 
            fprintf(fid, ';\n');
        end
    end
    for m=1:N    % 再应用对称性计算P的上/下三角
        if up_first==1 k1=m+1; k2=N; else k1=1; k2=m-1; end
        for n=k1:k2  
            fprintf(fid, '\tP[%2d*N+%2d]=P[%2d*N+%2d];', m-1,n-1, n-1,m-1);   
        end  
        fprintf(fid, '\n');
    end
    % P1 = P1 + Q*Ts
    fprintf(fid, '\t//P = P + Q*Ts\n'); 
    fprintf(fid, '\t'); 
    for m=1:N 
        if Q(m,m)~=0 fprintf(fid, 'P[%2d*N+%2d]+=Q[%2d*N+%2d]*Ts; ', m-1,m-1, m-1,m-1); op_add = op_add+1; end
    end
    fprintf(fid, '\n');
    % X1 = (I+F)*X
    fprintf(fid, '\t//X1 = (I+F)*X\n'); 
    for m=1:N          
        fprintf(fid, '\tX1[%2d]=X[%2d]', m-1,m-1);
        for n=1:N
            if A(m,n)~=0    fprintf(fid, '+F[%2d*N+%2d]*X[%2d]', m-1,n-1,n-1); op_mul = op_mul+1;  end
        end
        fprintf(fid, ';\n');
    end
    fprintf(fid, '\tmemcpy(X, X1, N*sizeof(double));\n');
    fprintf(fid, '\t//op_mul+=%d, op_add+=%d;\n}\n\n', op_mul, op_add);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    op_mul = 0; op_add = 0;
    %%%%%%%%%%%%%%%% 子程序
    fprintf(fid, 'static void KXP(double *X, double *P, double *Pxz, double _pzz, double r)\n{\n'); 
    fprintf(fid, '\tdouble K[N];\n');
    % K = Pxz*Pzz^-1
    fprintf(fid, '\t//K = Pxz*Pzz^-1\n'); 
    for m=1:N
        fprintf(fid, '\tK[%2d]=Pxz[%2d]*_pzz;\n', m-1, m-1); op_mul = op_mul+1;
    end
    % X1 = X + K*Inno;
    fprintf(fid, '\t//X1 = X + K*Innovation\n'); 
    for m=1:N
        fprintf(fid, '\tX[%2d]+=K[%2d]*r;\n', m-1, m-1); op_add = op_add+1; op_mul = op_mul+1;
    end
    % P1 = P - K*Pxz^T;
    fprintf(fid, '\t//P1 = P - K*Pxz^T\n'); 
    for m=1:N    % 先计算下三角
        fprintf(fid, '\t');
        for n=m:N
            fprintf(fid, 'P[%2d*N+%2d]-=K[%2d]*Pxz[%2d]; ', m-1,n-1, m-1, n-1); 
            op_add = op_add+1; op_mul = op_mul+1;
        end
        fprintf(fid, '\n');
    end
    for m=1:N    % 再应用对称性计算P的上三角
        for n=1:m-1  
            fprintf(fid, '\tP[%2d*N+%2d]=P[%2d*N+%2d];', m-1,n-1, n-1,m-1);   
        end  
        fprintf(fid, '\n');
    end
    fprintf(fid, '}\n\n');
    op_add = op_add*M; op_mul = op_mul*M;
    %%%%%%%%%%%%%%%%
    fprintf(fid, 'double MeasUpdate(double *X, double *P, double *C, double *Z, double *R)\n{\n');
    fprintf(fid, '\tdouble Pxz[N], Pzz, _pzz, r, chi2=0.0;\n');
    for i=1:M
        Ci = C(i,:);  
        % Pxz = P*C^T
        fprintf(fid, '\t//Pxz = P*C(%d,:)^T\n',i-1); 
        for m=1:N           
            fprintf(fid, '\tPxz[%2d]=', m-1); 
            for n=1:N
                if Ci(n)==1    fprintf(fid, '+P[%2d*N+%2d]      ', m-1,n-1); op_add = op_add+1;
                elseif Ci(n)==-1    fprintf(fid, '-P[%2d*N+%2d]      ', m-1,n-1); op_add = op_add+1;
                elseif Ci(n)~=0    fprintf(fid, '+P[%2d*N+%2d]*C[%2d]', m-1,n-1,n-1);  op_mul = op_mul+1;
                end
            end 
            fprintf(fid, ';\n');
        end
        % Pzz = C*Pxz + R
        fprintf(fid, '\t//Pzz = C(%d,:)*Pxz + R\n',i-1); 
        fprintf(fid, '\tPzz=');
        for m=1:N
            if Ci(m)==1 fprintf(fid, '      +Pxz[%2d]', m-1); op_add = op_add+1;
            elseif Ci(m)==-1 fprintf(fid, '      -Pxz[%2d]', m-1); op_add = op_add+1;
            elseif Ci(m)~=0 fprintf(fid, '+C[%2d]*Pxz[%2d]', m-1, m-1); op_mul = op_mul+1;
            end
        end
        fprintf(fid, '+R[%d];\n', (i-1)*M+i-1);
        % Inno = Z - C*X
        fprintf(fid, '\t//Innovation = Z(%d) - C(%d,:)*X\n',i-1,i-1); 
        fprintf(fid, '\tr=Z[%2d]',i-1);
        for m=1:N
            if Ci(m)==1 fprintf(fid, '      -X[%2d]', m-1); op_add = op_add+1;
            elseif Ci(m)==-1 fprintf(fid, '      +X[%2d]', m-1); op_add = op_add+1;
            elseif Ci(m)~=0 fprintf(fid, '-C[%2d]*X[%2d]', m-1, m-1); op_mul = op_mul+1;
            end
        end
        fprintf(fid, ';\n');
        fprintf(fid, '\t_pzz=1.0/Pzz;\n');
        fprintf(fid, '\tKXP(X, P, Pxz, _pzz, r);\n');
        fprintf(fid, '\tchi2+=r*_pzz*r;\n');
        if i<M
            fprintf(fid, '\n\tC += N;\n');
        end
    end
    fprintf(fid, '\treturn chi2;\n\t//op_mul+=%d, op_add+=%d;\n}\n', op_mul, op_add);
    fclose(fid);