% 마지막 시간 스텝 선택
for last_step = 1:length(pathx(:,:))

% 마지막 시간 스텝의 데이터 선택
    pathx_last = pathx(:,:,last_step);
    pathy_last = pathy(:,:,last_step);
    invalid_last = invalid(:,:,last_step);
    
    % 유효한 경로 인덱스 찾기
    valid_indices = find(invalid_last > 0);
    
    % 유효한 경로 데이터 선택
    pathx_valid = pathx_last(valid_indices);
    pathy_valid = pathy_last(valid_indices);
%     x = pathx_valid(3);
%     y = pathy_valid(2);    
%     x2 = pathx_valid(1);
%     y2 = pathy_valid(1);
    % 그래프 그리기
    figure(3);
    hold on
    plot(pathx_valid, pathy_valid, 'LineWidth', 1);
%     plot(x,y,'*','LineWidth', 1)
%     plot(x2,y2,'o','LineWidth', 1)
    xlabel('X');
    ylabel('Y');
    title('Valid Path');
    grid on;
end