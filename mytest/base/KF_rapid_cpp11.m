function KF_rapid_cpp11(fname, A, C, Q, seq)
% 快速滤波计算法：根据矩阵A、C和Q的非零元素直接展开，生成C语言程序，参见:
%   "A rapid computation method for Kalman filtering in vehicular SINS/GPS integrated system"
% 滤波模型：dXt = At*Xt+Qt, Zk = Ck*Xk+Rk
%   A--连续时间系统的一步转移矩阵, C--量测矩阵, Q--系统噪声（需对角阵）, R--量测噪声（需对角阵）
    [M,N] = size(C);
    sh = sprintf('%s.h', fname);
    fid = fopen(sh,'wt');
    fprintf(fid, '#ifndef _%s_h\n',fname);
    fprintf(fid, '#define _%s_h\n\n',fname);
    fprintf(fid, 'double* IATs(double *A, double Ts);\n');
    fprintf(fid, 'void SysUpdate(double *X, double *P, double *A, double *Q);\n');
    fprintf(fid, 'void MeasUpdate(double *X, double *P, double *C, double *Z, double *R);\n\n');
    fprintf(fid, '#endif\n');
    fclose(fid);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    scpp = sprintf('%s.cpp', fname);
    fid = fopen(scpp,'wt');
    fprintf(fid, '#include "stdAfx.h"\n');
    fprintf(fid, '#include "navi.h"\n');
    fprintf(fid, '#include "%s"\n\n',sh);
    fprintf(fid, '#define M %d\n', M);
    fprintf(fid, '#define N %d\n\n', N);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    op_mul = 0; op_add = 0; % 乘法、加法的运算次数
    fprintf(fid, 'double* IATs(double *A, double Ts)\n{\n');
    % Ak = At*Ts
    fprintf(fid, '\t//Ak = At*Ts\n');
    for m=1:N
        for n=1:N
            if A(m,n)~=0 fprintf(fid, '\tA[%2d*N+%2d]*=Ts;', m-1,n-1); op_mul = op_mul+1; end
        end
        if sum(abs(A(m,:)))>0  fprintf(fid, '\n'); end
    end
    % Ak = At*Ts
    fprintf(fid, '\t//Ak = I+At*Ts\n');
    for m=1:N
        if A(m,m)==0 fprintf(fid, '\tA[%2d*N+%2d]=1.0;\n', m-1,m-1); 
        else fprintf(fid, '\tA[%2d*N+%2d]+=1.0;\n', m-1,m-1); op_add = op_add+1; end
    end
    fprintf(fid, '\treturn A;\n');
    fprintf(fid, '\t//op_mul+=%d, op_add+=%d;\n}\n\n', op_mul, op_add);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    op_mul = 0; op_add = 0; 
    A = A*(pi^1.123) + eye(N); % 为区分1元素
    fprintf(fid, 'void SysUpdate(double *X, double *P, double *A, double *Q)\n{\n');
    fprintf(fid, '\tdouble B[N*N], X1[N];\n');
    % B = Ak*P
    fprintf(fid, '\t//B = A*P\n'); 
    for m=1:N          
        for n=1:N
            fprintf(fid, '\tB[%2d*N+%2d]=', m-1,n-1); % B(m,n)=
            for k=1:N
                if A(m,k)==1    fprintf(fid, '+P[%2d*N+%2d]', k-1,n-1); op_add = op_add+1;
                elseif A(m,k)==-1   fprintf(fid, '-P[%2d*N+%2d]', k-1,n-1); op_add = op_add+1;
                elseif A(m,k)~=0    fprintf(fid, '+A[%2d*N+%2d]*P[%2d*N+%2d]', m-1,k-1,k-1,n-1); op_mul = op_mul+1;
                end
            end
            fprintf(fid, ';\n');
        end
    end
    % P = B*Ak^T
    fprintf(fid, '\t//P = B*A^T\n'); 
    down_no0 = 0; up_no0 = 0; % 比较A的上或下三角非零元素多少
    for m=1:N
        for n=1:N
            if m>n & A(m,n)~=0 down_no0 = down_no0+1; end
            if m<n & A(m,n)~=0 up_no0 = up_no0+1; end
        end
    end
    for m=1:N 
        if up_no0<down_no0  k1 = 1; k2 = m;  else  k1=m; k2=N;  end
        for n=k1:k2       % 先计算下/上三角
            fprintf(fid, '\tP[%2d*N+%2d]=', m-1,n-1); 
            for k=1:N
                if A(n,k)==1    fprintf(fid, '+B[%2d*N+%2d]', m-1,k-1); op_add = op_add+1;
                elseif A(n,k)==-1   fprintf(fid, '-B[%2d*N+%2d]', m-1,k-1); op_add = op_add+1;
                elseif A(n,k)~=0    fprintf(fid, '+B[%2d*N+%2d]*A[%2d*N+%2d]', m-1,k-1,n-1,k-1);  op_mul = op_mul+1;
                end
            end 
            fprintf(fid, ';\n');
        end
    end
    for m=1:N    % 再应用对称性计算P的上/下三角
        if up_no0<down_no0 k1=m+1; k2=N; else k1=1; k2=m-1; end
        for n=k1:k2  
            fprintf(fid, '\tP[%2d*N+%2d]=P[%2d*N+%2d];', m-1,n-1, n-1,m-1);   
        end  
        fprintf(fid, '\n');
    end
    fprintf(fid, '\t//P = P + Q\n'); 
    % P = P + Q
    fprintf(fid, '\t'); 
    for m=1:N 
        if Q(m,m)~=0 fprintf(fid, 'P[%2d*N+%2d]+=Q[%2d*N+%2d]; ', m-1,m-1, m-1,m-1); op_add = op_add+1; end
    end
    fprintf(fid, '\n');
    % X1 = Ak*X
    fprintf(fid, '\t//X1 = A*X\n'); 
    for m=1:N          
        fprintf(fid, '\tX1[%2d]=', m-1);
        for n=1:N
            if A(m,n)==1    fprintf(fid, '           +X[%2d]', n-1);
            elseif A(m,n)==-1    fprintf(fid, '           -X[%2d]', n-1);
            elseif A(m,n)~=0    fprintf(fid, '+A[%2d*N+%2d]*X[%2d]', m-1,n-1,n-1); op_mul = op_mul+1;
            end
        end
        fprintf(fid, ';\n');
    end
    fprintf(fid, '\tmemcpy(X, X1, N*sizeof(double));\n');
    fprintf(fid, '\t//op_mul+=%d, op_add+=%d;\n}\n\n', op_mul, op_add);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    op_mul = 0; op_add = 0;
if seq==1  % 序贯滤波法
    %%%%%%%%%%%%%%%% 子程序
    fprintf(fid, 'static void KXP(double *X, double *P, double *K, double *Pxz, double Pzz, double Xnew)\n{\n'); 
    % K = Pxz*Pzz^-1
    fprintf(fid, '\tdouble _pxz=1.0/Pzz;\n');
    fprintf(fid, '\t//K = Pxz*Pzz^-1\n'); 
    for m=1:N
        fprintf(fid, '\tK[%2d]=Pxz[%2d]*_pxz;\n', m-1, m-1); op_mul = op_mul+1;
    end
    % X1 = X + K*Xnew;
    fprintf(fid, '\t//X1 = X + K*Xnew\n'); 
    for m=1:N
        fprintf(fid, '\tX[%2d]+=K[%2d]*Xnew;\n', m-1, m-1); op_add = op_add+1; op_mul = op_mul+1;
    end
    % P1 = P - K*Pxz^T;
    fprintf(fid, '\t//P1 = P - K*Pxz^T\n'); 
    for m=1:N
        fprintf(fid, '\t');
        for n=1:N
            fprintf(fid, 'P[%2d*N+%2d]-=K[%2d]*Pxz[%2d]; ', m-1,n-1, m-1, n-1); 
            op_add = op_add+1; op_mul = op_mul+1;
        end
        fprintf(fid, '\n');
    end
    fprintf(fid, '}\n\n');
    op_add = op_add*M; op_mul = op_mul*M;
    %%%%%%%%%%%%%%%%
    fprintf(fid, 'void MeasUpdate(double *X, double *P, double *C, double *Z, double *R)\n{\n');
    fprintf(fid, '\tdouble Pxz[N], K[N], Pzz, Xnew;\n');
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
        % Xnew = Z - C*X
        fprintf(fid, '\t//Xnew = Z(%d) - C(%d,:)*X\n',i-1,i-1); 
        fprintf(fid, '\tXnew=Z[%2d]',i-1);
        for m=1:N
            if Ci(m)==1 fprintf(fid, '      -X[%2d]', m-1); op_add = op_add+1;
            elseif Ci(m)==-1 fprintf(fid, '      +X[%2d]', m-1); op_add = op_add+1;
            elseif Ci(m)~=0 fprintf(fid, '-C[%2d]*X[%2d]', m-1, m-1); op_mul = op_mul+1;
            end
        end
        fprintf(fid, ';\n');
        fprintf(fid, '\tKXP(X, P, K, Pxz, Pzz, Xnew);\n');
        if i<M
            fprintf(fid, '\n\tC += N;\n');
        end
    end
else % 非序贯滤波法
    fprintf(fid, 'void MeasUpdate(double *X, double *P, double *C, double *Z, double *R)\n{\n');
    fprintf(fid, '\tdouble Pxz[N*M], K[N*M], Pzz[M*M], Xnew[M];\n');
    % Pxz = P*C^T
    fprintf(fid, '\t//Pxz = P*C^T\n'); 
    CT = C';   % !!!
    for m=1:N           
        for n=1:M
            fprintf(fid, '\tPxz[%2d*M+%2d]=', m-1,n-1); 
            for k=1:N
                if CT(k,n)==1    fprintf(fid, '+P[%2d*N+%2d]', m-1,k-1); op_add = op_add+1;
                elseif CT(k,n)==-1   fprintf(fid, '-P[%2d*N+%2d]', m-1,k-1); op_add = op_add+1;
                elseif CT(k,n)~=0    fprintf(fid, '+P[%2d*N+%2d]*C[%2d*N+%2d]', m-1,k-1,n-1,k-1);  op_mul = op_mul+1;
                end
            end 
            fprintf(fid, ';\n');
        end
    end
    % Pzz = C*Pxz
    fprintf(fid, '\t//Pzz = C*Pxz\n'); 
    for m=1:M          
        for n=1:M
            fprintf(fid, '\tPzz[%2d*M+%2d]=', m-1,n-1); % Pzz(m,n)=
            for k=1:N
                if C(m,k)==1    fprintf(fid, '+Pxz[%2d*M+%2d]', k-1,n-1); op_add = op_add+1;
                elseif C(m,k)==-1   fprintf(fid, '-Pxz[%2d*M+%2d]', k-1,n-1); op_add = op_add+1;
                elseif C(m,k)~=0    fprintf(fid, '+C[%2d*N+%2d]*Pxz[%2d*M+%2d]', m-1,k-1,k-1,n-1); op_mul = op_mul+1;
                end
            end
            fprintf(fid, ';\n');
        end
    end
    % Pzz = Pzz + R
    fprintf(fid, '\t'); 
    for m=1:M   
        fprintf(fid, 'Pzz[%2d*M+%2d]+=R[%2d*M+%2d]; ', m-1,m-1, m-1,m-1);
    end
    fprintf(fid, '\n'); 
    % K = Pxz*Pzz^-1
    fprintf(fid, '\t//K = Pxz*Pzz^-1\n'); 
    fprintf(fid, '\tbrinv(Pzz, M);\n');
    for m=1:N          
        for n=1:M
            fprintf(fid, '\tK[%2d*M+%2d]=', m-1,n-1); % K(m,n)=
            for k=1:M
                fprintf(fid, '+Pxz[%2d*M+%2d]*Pzz[%2d*M+%2d]', m-1,k-1,k-1,n-1); op_mul = op_mul+1;
            end
            fprintf(fid, ';\n');
        end
    end
    % Xnew = Z - C*X
    fprintf(fid, '\t//Xnew = Z - C*X\n'); 
    for m=1:M
        fprintf(fid, '\tXnew[%2d]=Z[%2d]',m-1,m-1);
        for n=1:N
            if C(m,n)==1 fprintf(fid, '      -X[%2d]', n-1); op_add = op_add+1;
            elseif C(m,n)==-1 fprintf(fid, '      +X[%2d]', n-1); op_add = op_add+1;
            elseif C(m,n)~=0 fprintf(fid, '-C[%2d*N+%2d]*X[%2d]', m-1,n-1, n-1); op_mul = op_mul+1;
            end
        end
        fprintf(fid, ';\n');
    end
    % X1 = X + K*Xnew;
    fprintf(fid, '\t//X1 = X + K*Xnew\n'); 
    for m=1:N
        fprintf(fid, '\tX[%2d]+=', m-1);
        for n=1:M
            fprintf(fid, '+K[%2d*M+%2d]*Xnew[%2d]', m-1,n-1, n-1); op_add = op_add+1; op_mul = op_mul+1;
        end
        fprintf(fid, ';\n');
    end
    % P1 = P - K*Pxz^T
    fprintf(fid, '\t//P1 = P - K*Pxz^T\n'); 
    for m=1:N           
        for n=1:N
            fprintf(fid, '\tP[%2d*N+%2d]-=', m-1,n-1); 
            for k=1:M
                fprintf(fid, '+K[%2d*M+%2d]*Pxz[%2d*M+%2d]', m-1,k-1,n-1,k-1);  op_mul = op_mul+1;
            end 
            fprintf(fid, ';\n');
        end
    end
end
    fprintf(fid, '\t//op_mul+=%d, op_add+=%d;\n}\n', op_mul, op_add);
    fclose(fid);