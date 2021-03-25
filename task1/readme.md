Nalaganje potrebnih paketov:

sudo apt update
sudo apt install python3-pip
pip3 install dlib


Zalaufaj v konzoli (v istem direktoriju kot je skritpa in datoteke): python3 face_detector_test.py

Pritisni q za izhod



Dokumentacija:

face_finder: glavni razred
  find_faces(rgb_image): Prikaže podano sliko v oknu, nariše zelene pravokotnike okoli zaznanih obrazov
  process_image_from_file(image_path): Odpre sliko iz podane poti in jo posreduje metodi find_faces()
  display_from_camera(): V živo zajema sliko iz vgrajene kamere (testirano samo z laptop kamero) in pošilja posamezne frame metodi find_faces. Uporabno za hitro testiranje.
