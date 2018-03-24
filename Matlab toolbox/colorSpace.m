function colorSpace = choosedialog

    d = dialog('Position',[500 250 250 150],'Name','Select One');
    txt = uicontrol('Parent',d,...
           'Style','text',...
           'Position',[20 80 210 40],...
           'String','Select the color space');
       
    popup = uicontrol('Parent',d,...
           'Style','popup',...
           'Position',[75 70 100 25],...
           'String',{'RGB2NTSC';'RGB2YCBR';'RGB2GRAY';'RGB2HSV'},...
           'Callback',@popup_callback);
       
    btn = uicontrol('Parent',d,...
           'Position',[89 20 70 25],...
           'String','Ok',...
           'Callback','delete(gcf)');
       
    colorSpace = 1;
       
    % Wait for d to close before running to completion
    uiwait(d);
   
       function popup_callback(popup,event)
          colorSpace = popup.Value;
%           popup_items = popup.String;
          % This code uses dot notation to get properties.
          % Dot notation runs in R2014b and later.
          % For R2014a and earlier:
          % idx = get(popup,'Value');
          % popup_items = get(popup,'String');
%           morphOperation = char(popup_items(idx,:));
       end
end