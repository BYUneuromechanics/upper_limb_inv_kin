function G = MatMult3(A, B, C, D)

% Matrices A and B are rotation matrices at calibration and
% therefore have size (3,3,1). In contrast, matrices C and D are
% rotation matrices at time t and therefore have size (3,3,length(t)).

G = zeros(size(C));

for i = 1:size(C,3)
    
    E = C(:,:,i)*A;
    F = D(:,:,i)*B;
    G(:,:,i) = E'*F;
    
    % This specific method of multiplication of matricies can be shown
    % through an example to show implementation.
    % Example: (REU(t)*RAE)'*(RFU(t)*RBF) = RBA(t) ; This is also the same
    % as RUA(t)*RBU(t) = RBA(t), where (REU(t)*RAE)' = RUA(t) and
    % (RFU(t)*RBF) = RBU(t).
    
end