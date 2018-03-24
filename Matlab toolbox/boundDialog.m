function filter = boundDialog

    prompt = {'Enter thrshold value', 'Enter the number of top areas to draw'};
     num_lines =1;
     dlg_title = 'Bounding box';
     defaultans = {'100', '5'};
     filter = inputdlg(prompt,dlg_title,num_lines,defaultans);  
     
end