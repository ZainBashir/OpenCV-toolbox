function filter = circlesDialog

    prompt = {'Enter the number of strongest circles you want to find'};
     num_lines =1;
     dlg_title = 'Circles';
     defaultans = {'1'};
     filter = inputdlg(prompt,dlg_title,num_lines,defaultans);  
     

end
