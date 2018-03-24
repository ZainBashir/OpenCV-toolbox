function filter = openDialog

    prompt = {'Enter size of square structuring element'};
     num_lines =1;
     dlg_title = 'Opening';
     defaultans = {'4'};
     filter = inputdlg(prompt,dlg_title,num_lines,defaultans);  
     
end