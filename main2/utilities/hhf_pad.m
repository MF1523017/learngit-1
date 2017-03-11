function [ P ] = hhf_pad( I )
    h = size(I, 1);
    w = size(I, 2);
    
    % Pad top and bottom
    %P = [(2*I(1,:,:)-I(3,:,:)); (2*I(1,:,:)-I(2,:,:)); I; (2*I(h,:,:)-I(h-1,:,:)); (2*I(h,:,:)-I(h-2,:,:))];
    P = [I(1,:,:); I(2,:,:); I; I(h-1,:,:); I(h,:,:)];
    
    % Pad left and right
    P = [P(:,1,:) P(:,2,:) P P(:,w-1,:) P(:,w,:)];
end

