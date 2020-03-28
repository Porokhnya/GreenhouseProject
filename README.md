
<h1>Introduction/Вступление</h1>

Arduino based smart greenhouse - is the project, intended to provide fully automatic way to control your greenhouse with many tasty features, like MQTT, Wi-Fi, Web-based interface, IoT (ThingSpeak, GardenBoss.ru), configuration software and so on.

Умная теплица на Arduino - проект прошивки, позволяющей вам полностью автоматизировать вашу теплицу, и включающий в себя многие вкусные плюшки, такие как поддержка MQTT, работу через Wi-Fi, работу с IoT (ThingSpeak, GardenBoss.ru), веб-интерфейс, конфигуратор, и многое другое.


<h1>NB: to English readers</h1>
<p>
Need volunteers to help with translating the interface into English (configurator, web-interface, firmware settings, project description on GitHub, and so on). For details, please write to <a href="mailto:spywarrior@gmail.com">spywarrior@gmail.com</a> - English is not my native language, I can translate from Russian to it, but with permanently bad results :(.

<h1>About/О проекте</h1>

<ul>
 <li><a href="README_EN.md">English description of the project</a></li>
 <li><a href="README_RU.md">Русская версия описания проекта</a></li>
</ul>

<h1>Links/Ресурсы</h1>
<ul>
 <li>Cайт проекта: <a href="http://gardenboss.ru" target="_blank">gardenboss.ru</a></li>
 <li>Website: <a href="http://gardenboss.ru" target="_blank">gardenboss.ru</a></li>
 <li>Приложение под Android: <a href="https://play.google.com/store/apps/details?id=ru.gardenboss.controller" target="_blank">Gardenboss Controller</li>
 <li>Android app: <a href="https://play.google.com/store/apps/details?id=ru.gardenboss.controller" target="_blank">Gardenboss Controller</li>
</ul>

<h1>Расширенная версия проекта</h1>

Расширенная версия проекта обеспечивает (без мороки с правилами), следующий дополнительный функционал:

<ul>
<li>Автоматическое управление окнами по секциям, по скорости и направлению ветра, по датчику дождя, по ориентации окон по сторонам света;
<li>Автоматическое управление отоплением (3 контура);
<li>Автоматическое управление затенением (3 контура);
<li>Автоматическое управление досветкой;
<li>Автоматическое управление циркуляционной вентиляцией (3 канала);
<li>Автоматическое управление вентиляцией по температуре (3 канала);
<li>Автоматическая настройка EEPROM перед первым запуском в работу;
<li>Работа с датчиками скорости и направления ветра, а также с датчиком дождя - через выносной модуль по RS-485 и радиоканалам (если не хочется пришпиливать к контроллеру);
<li>Отображение атмосферного давления;
<li>Удобное и быстрое перелистывание параметров путём зажатия и удержания экранной кнопки;
<li>Настройки синхронизации времени через ESP и SIM800 - на TFT-экране, в отличие от жёстко прошитых в открытой версии;
 <li>Возможность использования любого оператора GSM, в отличие от ограниченного количества указанных в прошивке открытой версии;
<li>Настройки параметров полива через TFT-экран, не прибегая к конфигуратору.
</ul>
 
 Для запроса расширенной версии перейдите в папку "Расширенная версия" и следуйте инструкциям в файле "Инструкции.txt"
