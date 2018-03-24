function filter = blurDialog

    prompt = {'Enter sigma value','Enter filter size'};
     num_lines =1;
     dlg_title = 'Blur';
     defaultans = {'1','3'};
     filter = inputdlg(prompt,dlg_title,num_lines,defaultans);  
     
end