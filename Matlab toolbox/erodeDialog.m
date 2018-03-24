function filter = erodeDialog

  prompt = {'Enter width for the square structuring element'};
     num_lines =1;
     dlg_title = 'Erode';
     defaultans = {'4'};
     filter = inputdlg(prompt,dlg_title,num_lines,defaultans); 
end