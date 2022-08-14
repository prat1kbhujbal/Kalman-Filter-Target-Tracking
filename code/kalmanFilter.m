function [ predictx, predicty, state, param ] = kalmanFilter( current_t, x, y, state, param, last_t )
    if last_t<0
        state = [x, y, 0, 0];
        param.P = 2 * eye(4);
        predictx = x;
        predicty = y;
        return;
    end
    dt =current_t - last_t;
    A = [1 0 dt 0;   
         0 1 0 dt;
         0 0 1 0;
         0 0 0 1];
    C = [1 0 0 0;
         0 1 0 0];
    z = [x,y].';
    snoise_cov = [dt*dt/4  0    dt/2 0 ;
               0    dt*dt/4  0  dt/2;
               dt/2     0    1   0;
               0     dt/2     0  1];
    
    mnoise_cov = [0.015, 0;
               0, 0.015];
    P = A * param.P * transpose(A) +  snoise_cov;
    R = mnoise_cov;
    K = P * transpose(C) * inv(R + C * P * transpose(C));
    predict = A * (state.') + K*(z - C * A * (state.'));
    param.P = P - K * C * P;
    state = predict.';
    predictx = predict(1) + predict(3)*(0.330);
    predicty = predict(2) + predict(4)*(0.330);
end
