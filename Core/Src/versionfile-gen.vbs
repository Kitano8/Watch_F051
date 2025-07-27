Dim Args
Set Args = WScript.Arguments

if (Args.Count < 1) then
  MsgBox "Error generate version file." + vbCrLf + "Usage:" + vbCrLf + "version-gen.vbs <filename>"
else
  'получаем имя выходного файла
  verfilename = Args(0)
  'получаем дату/время
  vYear  = CStr(Year  (Date))
  vMonth = CStr(Month (Date))
  if Len(vMonth) < 2 then
     vMonth = "0"+vMonth
  end if
  vDay    = CStr(Day    (Date))
  if Len(vDay) < 2 then
     vDay = "0"+vDay
  end if
  vHour  = CStr(Hour  (Time))
  if Len(vHour) < 2 then
     vHour = "0"+vHour
  end if
  vMinute = CStr(Minute (Time))
  if Len(vMinute) < 2 then
     vMinute = "0"+vMinute
  end if
  vSecond = CStr(Second (Time))
  if Len(vSecond) < 2 then
     vSecond = "0"+vSecond
  end if
  vWDay = CStr(Weekday(Date))
  version_full = "const char version_full [] = """+vDay+"."+vMonth+"."+vYear+"."+vWDay+""";"
  version_logo = "const char version_logo [] = """+vHour+"."+vMinute+"."+vSecond+""";"
  Set FSO = CreateObject("Scripting.FileSystemObject")
  Set FileOutStream = FSO.OpenTextFile(verfilename, 2, true, 0)
  FileOutStream.Write version_full + vbCrLf
  FileOutStream.Write version_logo + vbCrLf
end if